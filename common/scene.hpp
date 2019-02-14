#pragma once
#include <embree3/rtcore.h>
#include "material.hpp"
#include "assertion.hpp"

namespace rt {
	class MaterialDeclaration {
	public:
		virtual ~MaterialDeclaration() {}
		virtual const char *name() const = 0;
		virtual std::unique_ptr<BxDF> instanciate(const houdini_alembic::PolygonMeshObject *p, uint32_t primitive_index, const glm::dmat3 &xformInverseTransposed) const = 0;
	};
	class LambertianDeclaration : public MaterialDeclaration {
	public:
		const char *name() const override {
			return "Lambertian";
		}
		std::unique_ptr<BxDF> instanciate(const houdini_alembic::PolygonMeshObject *p, uint32_t primitive_index, const glm::dmat3 &xformInverseTransposed) const override {
			std::unique_ptr<LambertianBRDF> bxdf(new LambertianBRDF());
			if (auto Cd = p->primitives.column_as_vector3("Cd")) {
				Cd->get(primitive_index, glm::value_ptr(bxdf->R));
			}
			if (auto Le = p->primitives.column_as_vector3("Le")) {
				Le->get(primitive_index, glm::value_ptr(bxdf->Le));
			}
			if (auto backEmission = p->primitives.column_as_int("back_emission")) {
				bxdf->backEmission = backEmission->get(primitive_index) != 0;
			}
			return bxdf;
		}
	};
	static std::vector<MaterialDeclaration *> MaterialDeclarations = {
		new LambertianDeclaration(),
	};

	inline std::vector<std::unique_ptr<BxDF>> instanciateMaterials(houdini_alembic::PolygonMeshObject *p, const glm::dmat3 &xformInverseTransposed) {
		std::vector<std::unique_ptr<BxDF>> materials;

		auto material_string = p->primitives.column_as_string("material");
		if (material_string == nullptr) {
			return materials;
		}

		for (uint32_t i = 0, n = p->primitives.rowCount(); i < n; ++i) {
			std::unique_ptr<BxDF> mat;

			const std::string m = material_string->get(i);
			for (int j = 0; j < MaterialDeclarations.size(); ++j) {
				if (m == MaterialDeclarations[j]->name()) {
					mat = MaterialDeclarations[j]->instanciate(p, i, xformInverseTransposed);
					break;
				}
			}

			if (!mat) {
				mat = std::make_unique<LambertianBRDF>(glm::dvec3(), glm::dvec3(0.5), false);
			}

			materials.emplace_back(std::move(mat));
		}

		return materials;
	}

	inline void EmbreeErorrHandler(void* userPtr, RTCError code, const char* str) {
		printf("Embree Error [%d] %s\n", code, str);
	}

	// 平面の方程式 
	// ax + by + cz + d = 0
	// n = {a, b, c}
	struct PlaneEquation {
		glm::dvec3 n;
		double d = 0.0;

		void from_point_and_normal(glm::dvec3 point_on_plane, glm::dvec3 normalized_normal) {
			d = -glm::dot(point_on_plane, normalized_normal);
			n = normalized_normal;
		}
		double signed_distance(glm::dvec3 p) const {
			return glm::dot(n, p) + d;
		}
	};

	inline glm::dvec3 triangle_normal(const glm::dvec3 &v0, const glm::dvec3 &v1, const glm::dvec3 &v2) {
		glm::dvec3 e1 = v1 - v0;
		glm::dvec3 e2 = v2 - v0;
		return glm::normalize(glm::cross(e1, e2));
	}

	struct Luminaire {
		glm::dvec3 points[3];
		glm::dvec3 Ng;
		bool backenable = false;
		PlaneEquation plane;
	};
	class Scene {
	public:
		Scene(std::shared_ptr<houdini_alembic::AlembicScene> scene) : _scene(scene) {
			_embreeDevice = std::shared_ptr<RTCDeviceTy>(rtcNewDevice("set_affinity=1"), rtcReleaseDevice);
			rtcSetDeviceErrorFunction(_embreeDevice.get(), EmbreeErorrHandler, nullptr);

			_embreeScene = std::shared_ptr<RTCSceneTy>(rtcNewScene(_embreeDevice.get()), rtcReleaseScene);
			rtcSetSceneBuildQuality(_embreeScene.get(), RTC_BUILD_QUALITY_HIGH);
			
			for (auto o : scene->objects) {
				if (o->visible == false) {
					continue;
				}

				if (_camera == nullptr) {
					if (auto camera = o.as_camera()) {
						_camera = camera;
					}
				}

				if (auto polymesh = o.as_polygonMesh()) {
					addPolymesh(polymesh);
				}
			}
			RT_ASSERT(_camera);

			rtcCommitScene(_embreeScene.get());
			rtcInitIntersectContext(&_context);
		}
		
		Scene(const Scene &) = delete;
		void operator=(const Scene &) = delete;

		bool intersect(const glm::dvec3 &ro, const glm::dvec3 &rd, ShadingPoint *shadingPoint, float *tmin) const {
			RTCRayHit rayhit;
			rayhit.ray.org_x = ro.x;
			rayhit.ray.org_y = ro.y;
			rayhit.ray.org_z = ro.z;
			rayhit.ray.dir_x = rd.x;
			rayhit.ray.dir_y = rd.y;
			rayhit.ray.dir_z = rd.z;
			rayhit.ray.time = 0.0f;

			rayhit.ray.tfar = FLT_MAX;
			rayhit.ray.tnear = 0.0f;

			rayhit.ray.mask = 0;
			rayhit.ray.id = 0;
			rayhit.ray.flags = 0;
			rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
			rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
			rtcIntersect1(_embreeScene.get(), &_context, &rayhit);

			if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
				return false;
			}

			*tmin = rayhit.ray.tfar;

			int index = rayhit.hit.geomID;
			RT_ASSERT(index < _polymeshes.size());
			const Polymesh *mesh = _polymeshes[index].get();

			RT_ASSERT(rayhit.hit.primID < mesh->materials.size());
			shadingPoint->bxdf = mesh->materials[rayhit.hit.primID].get();

			shadingPoint->Ng.x = rayhit.hit.Ng_x;
			shadingPoint->Ng.y = rayhit.hit.Ng_y;
			shadingPoint->Ng.z = rayhit.hit.Ng_z;
			shadingPoint->u = rayhit.hit.u;
			shadingPoint->v = rayhit.hit.v;

			/*
			https://embree.github.io/api.html
			t_uv = (1-u-v)*t0 + u*t1 + v*t2
			= t0 + u*(t1-t0) + v*(t2-t0)
			*/
			//double u = rayhit.hit.u;
			//double v = rayhit.hit.v;
			//auto v0 = geom.points[prim.indices[0]].P;
			//auto v1 = geom.points[prim.indices[1]].P;
			//auto v2 = geom.points[prim.indices[2]].P;
			//(*material)->p = (1.0 - u - v) * v0 + u * v1 + v * v2;

			return true;
		}

		houdini_alembic::CameraObject *camera() {
			return _camera;
		}
		const std::vector<Luminaire> &luminaires() const {
			return _luminaires;
		}
	private:
		class Polymesh {
		public:
			std::vector<std::unique_ptr<BxDF>> materials;
			std::vector<uint32_t> indices;
			std::vector<glm::vec3> points;
		};

		void addPolymesh(houdini_alembic::PolygonMeshObject *p) {
			bool isTriangleMesh = std::all_of(p->faceCounts.begin(), p->faceCounts.end(), [](int32_t f) { return f == 3; });
			if (isTriangleMesh == false) {
				printf("skipped non-triangle mesh: %s\n", p->name.c_str());
				return;
			}

			std::unique_ptr<Polymesh> polymesh(new Polymesh());
			polymesh->indices = p->indices;

			// Vertexのアトリビュートを読むときにも影響を受けるので注意が必要
			// Houdini (CW) => (CCW)
			for (int i = 0; i < polymesh->indices.size(); i += 3) {
				std::swap(polymesh->indices[i + 1], polymesh->indices[i + 2]);
			}

			RT_ASSERT(std::all_of(polymesh->indices.begin(), polymesh->indices.end(), [p](uint32_t index) { return index < p->points.rowCount(); }));

			glm::dmat4 xform;
			for (int i = 0; i < 16; ++i) {
				glm::value_ptr(xform)[i] = p->combinedXforms.value_ptr()[i];
			}
			glm::dmat3 xformInverseTransposed = glm::inverseTranspose(xform);

			polymesh->points.reserve(p->P.size());
			for (auto srcP : p->P) {
				glm::vec3 p = xform * glm::dvec4(srcP.x, srcP.y, srcP.z, 1.0);
				polymesh->points.emplace_back(p);
			}

			polymesh->materials = instanciateMaterials(p, xformInverseTransposed);

			// add Luminaire
			auto luminaires = p->primitives.column_as_int("luminaires");
			auto luminaires_backenable = p->primitives.column_as_int("luminaires_backenable");
			if (luminaires && luminaires_backenable) {
				RT_ASSERT(luminaires->rowCount() == p->primitives.rowCount());
				RT_ASSERT(luminaires_backenable->rowCount() == p->primitives.rowCount());

				for (int i = 0; i < p->primitives.rowCount(); ++i) {
					if (luminaires->get(i)) {
						Luminaire L;
						for (int j = 0; j < 3; ++j) {
							int index_src = i * 3 + j;
							RT_ASSERT(index_src < polymesh->indices.size());
							int index = polymesh->indices[index_src];
							RT_ASSERT(index < polymesh->points.size());
							L.points[j] = polymesh->points[index];
						}
						L.backenable = luminaires_backenable->get(i) != 0;
						L.Ng = triangle_normal(L.points[0], L.points[1], L.points[2]);
						L.plane.from_point_and_normal(L.points[0], L.Ng);
						_luminaires.emplace_back(L);
					}
				}
			}

			// add to embree
			// https://www.slideshare.net/IntelSoftware/embree-ray-tracing-kernels-overview-and-new-features-siggraph-2018-tech-session
			RTCGeometry g = rtcNewGeometry(_embreeDevice.get(), RTC_GEOMETRY_TYPE_TRIANGLE);

			size_t vertexStride = sizeof(glm::vec3);
			rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_VERTEX, 0 /*slot*/, RTC_FORMAT_FLOAT3, polymesh->points.data(), 0 /*byteoffset*/, vertexStride, polymesh->points.size());
			
			size_t indexStride = sizeof(uint32_t) * 3;
			size_t primitiveCount = polymesh->indices.size() / 3;
			rtcSetSharedGeometryBuffer(g, RTC_BUFFER_TYPE_INDEX, 0 /*slot*/, RTC_FORMAT_UINT3, polymesh->indices.data(), 0 /*byteoffset*/, indexStride, primitiveCount);

			rtcCommitGeometry(g);
			rtcAttachGeometryByID(_embreeScene.get(), g, _polymeshes.size());
			rtcReleaseGeometry(g);

			// add to member
			_polymeshes.emplace_back(std::move(polymesh));
		}

	private:
		std::shared_ptr<houdini_alembic::AlembicScene> _scene;
		houdini_alembic::CameraObject *_camera = nullptr;
		std::vector<std::unique_ptr<Polymesh>> _polymeshes;

		std::shared_ptr<RTCDeviceTy> _embreeDevice;
		std::shared_ptr<RTCSceneTy> _embreeScene;

		std::vector<Luminaire> _luminaires;

		mutable RTCIntersectContext _context;
	};
}