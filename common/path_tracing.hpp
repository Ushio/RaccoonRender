#pragma once

#include <tbb/tbb.h>

#include "peseudo_random.hpp"
#include "material.hpp"
#include "scene.hpp"
#include "spherical_triangle_sampler.hpp"
#include "triangle_sampler.hpp"
#include "value_prportional_sampler.hpp"
#include "plane_equation.hpp"

namespace rt {
	class Image {
	public:
		Image(int w, int h) :_w(w), _h(h), _pixels(h * w), _randoms(h * w) {
			XoroshiroPlus128 random;
			for (int i = 0; i < _randoms.size(); ++i) {
				_randoms[i] = random;
				random.jump();
			}
		}
		int width() const {
			return _w;
		}
		int height() const {
			return _h;
		}

		void add(int x, int y, glm::dvec3 c) {
			int index = y * _w + x;
			_pixels[index].color += c;
			_pixels[index].sample++;
		}

		struct Pixel {
			int sample = 0;
			glm::dvec3 color;
		};
		const Pixel *pixel(int x, int y) const {
			return _pixels.data() + y * _w + x;
		}
		Pixel *pixel(int x, int y) {
			return _pixels.data() + y * _w + x;
		}

		PeseudoRandom *random(int x, int y) {
			return _randoms.data() + y * _w + x;
		}
	private:
		int _w = 0;
		int _h = 0;
		std::vector<Pixel> _pixels;
		std::vector<XoroshiroPlus128> _randoms;
	};



	class LuminaireSampler {
	public:
		void prepare(const std::vector<Luminaire> &luminaires, glm::dvec3 o) {
			_o = o;
			_selector.clear();
			_canSample = false;

			for (const Luminaire &L : luminaires) {
				// TODO 三角形が全部裏側のケース
				//double projected_area;
				//if (L.plane.signed_distance(o) < 1.0e-6) {
				//	// サンプルしようとする面に存在するケース
				//	projected_area = 0.0;
				//}
				//else {
				//	glm::dvec3 d = L.center - o;
				//	double distance_sqared = glm::length2(d);
				//	d /= std::sqrt(distance_sqared);
				//	projected_area = glm::abs(glm::dot(d, L.Ng)) * L.area / distance_sqared;
				//}
				//if (projected_area > 0.0) {
				//	_canSample = true;
				//}
				//_selector.add(projected_area);

				if (1.0e-6 < L.plane.signed_distance(o)) {
					_canSample = true;
					_selector.add(1.0);
				}
				else {
					_selector.add(0.0);
				}
			}
		}

		double pdf(const std::vector<Luminaire> &luminaires, glm::dvec3 wi) const {
			double p = 0.0;
			for (int i = 0; i < luminaires.size(); ++i) {
				double sP = _selector.probability(i);
				double tmin;
				if (0.0 < sP && intersect_ray_triangle(_o, wi, luminaires[i].points[0], luminaires[i].points[1], luminaires[i].points[2], &tmin)) {
					//SphericalTriangleSampler sSampler(luminaires[i].points[0], luminaires[i].points[1], luminaires[i].points[2], _o);
					//p += sP * (1.0 / sSampler.solidAngle());

					double pA = 1.0 / luminaires[i].area;
					double pW = pA * (tmin * tmin) / glm::abs(glm::dot(-wi, luminaires[i].Ng));
					p += sP * pW;
				}
			}
			return p;
		}
		glm::dvec3 sample(const std::vector<Luminaire> &luminaires, PeseudoRandom *random) const {
			int i = _selector.sample(random);
			//SphericalTriangleSampler sSampler(luminaires[i].points[0], luminaires[i].points[1], luminaires[i].points[2], _o);
			//return sSampler.sample_direction(random->uniform(), random->uniform());
			auto sampler = uniform_on_triangle(random->uniform(), random->uniform());
			auto p_on_triangle = sampler.evaluate(luminaires[i].points[0], luminaires[i].points[1], luminaires[i].points[2]);
			return glm::normalize(p_on_triangle - _o);
		}
		
		bool canSample() const {
			return _canSample;
		}
	private:
		bool _canSample = false;
		glm::dvec3 _o;
		ValueProportionalSampler<double> _selector;
	};

	class CailSampler {
	public:
		CailSampler(glm::dvec3 o):_o(o) {
			_plane.from_point_and_normal(glm::dvec3(0, 0.432329, 0), glm::dvec3(0, -1, 0));
		}
		glm::dvec3 sample(PeseudoRandom *random) const {
			glm::dvec3 p = glm::dvec3(
				random->uniform(-_size * 0.5, _size * 0.5),
				0.432329,
				random->uniform(-_size * 0.5, _size * 0.5)
			);

			return glm::normalize(p - _o);
		}
		double pdf(glm::dvec3 wi) const {
			if (canSample() == false) {
				return 0.0;
			}
			double tmin;
			if (_plane.intersect_ray(_o, wi, &tmin)) {
				glm::dvec3 p = _o + wi * tmin;
				
				if (-_size * 0.5 < p.x && p.x < _size * 0.5) {
					if (-_size * 0.5 < p.z && p.z < _size * 0.5) {
						double pA = 1.0 / (_size * _size);
						double pW = pA * (tmin * tmin) / glm::abs(glm::dot(-wi, _plane.n));
						return pW;
					}
				}
			}
			return 0.0;
		}
		bool canSample() const {
			// return 1.0e-6 < std::abs(_plane.signed_distance(_o));
			return 1.0e-3 < _plane.signed_distance(_o);
		}
		const double _size = 0.5;
		PlaneEquation _plane;
		glm::dvec3 _o;
	};

	inline glm::dvec3 radiance(const rt::Scene *scene, glm::dvec3 ro, glm::dvec3 rd, PeseudoRandom *random, int px, int py) {
		// const double kSceneEPS = scene.adaptiveEps();
		const double kSceneEPS = 1.0e-4;
		const double kValueEPS = 1.0e-6;

		glm::dvec3 Lo;
		glm::dvec3 T(1.0);

		int focuPixelx = 180;
		int focuPixely = 334;
		if (px == focuPixelx && py == focuPixely) {
			printf("");
		}

		// double mis_weight = 1.0;

		constexpr int kDepth = 10;
		for (int i = 0; i < kDepth; ++i) {
			float tmin = 0.0f;
			ShadingPoint shadingPoint;

			glm::dvec3 wo = -rd;

			if (scene->intersect(ro, rd, &shadingPoint, &tmin)) {
				auto p = ro + rd * (double)tmin;

				shadingPoint.Ng = glm::normalize(shadingPoint.Ng);
				glm::dvec3 wi;

				static thread_local LuminaireSampler directSampler;
				directSampler.prepare(scene->luminaires(), p);

				double P_Direct = directSampler.canSample() ? 0.5 : 0.0;
				//bool is_direct;

				if (random->uniform() < P_Direct) {
					//is_direct = true;
					wi = directSampler.sample(scene->luminaires(), random);
				}
				else {
					//is_direct = false;
					wi = shadingPoint.bxdf->sample(random, wo, shadingPoint);
				}

				double pdf_direct = directSampler.pdf(scene->luminaires(), wi);
				double pdf_bxdf = shadingPoint.bxdf->pdf(wo, wi, shadingPoint);

				if (directSampler.canSample() == false) {
					pdf_direct = 0.0;
				}
				double pdf = P_Direct * pdf_direct
					+ (1.0 - P_Direct) * pdf_bxdf;

				// もっともシンプルなやりかた。これだとうまくいく
				//CailSampler directSampler(p);
				//double P_Direct = directSampler.canSample() ? 0.7 : 0.0;
				//if (random->uniform() < P_Direct) {
				//	wi = directSampler.sample(random);
				//}
				//else {
				//	wi = shadingPoint.bxdf->sample(random, wo, shadingPoint);
				//}

				//double pdf_direct = directSampler.pdf(wi);
				//double pdf_bxdf = shadingPoint.bxdf->pdf(wo, wi, shadingPoint);

				//if (directSampler.canSample() == false) {
				//	pdf_direct = 0.0;
				//}
				//double pdf = P_Direct * pdf_direct
				//	+ (1.0 - P_Direct) * pdf_bxdf;
				//
				//RT_ASSERT(0.0 <= pdf);
				//RT_ASSERT(0.0 <= pdf_direct);
				//RT_ASSERT(0.0 <= pdf_bxdf);

				//if (p.y > 0.9) {
				//	RT_ASSERT(pdf_direct == 0.0);
				//}
	

				//if (std::isfinite(pdf) == false) {
				//	printf("");
				//}

				//----

				//if (is_direct) {
				//	if (pdf_direct == 0.0) {
				//		directSampler.pdf(scene->luminaires(), wi);
				//	}
				//}

				//double pdf;
				//double mis_weight;
				//auto sqr = [](double x) {
				//	return x * x;
				//};
				//if (is_direct) {
				//	mis_weight = sqr(pdf_direct) / (sqr(pdf_bxdf) + sqr(pdf_direct));
				//	pdf = P_Direct * pdf_direct;
				//}
				//else {
				//	mis_weight = sqr(pdf_bxdf) / (sqr(pdf_bxdf) + sqr(pdf_direct));
				//	pdf = (1.0 - P_Direct) * pdf_bxdf / mis_weight;
				//}


				// BSDFのみ
				//wi = shadingPoint.bxdf->sample(random, wo, shadingPoint);
				//double pdf = shadingPoint.bxdf->pdf(wo, wi, shadingPoint);

				glm::dvec3 bxdf = shadingPoint.bxdf->bxdf(wo, wi, shadingPoint);
				glm::dvec3 emission = shadingPoint.bxdf->emission(wo, shadingPoint);
				
				double NoI = glm::dot(shadingPoint.Ng, wi);
				double cosTheta = std::abs(NoI);

				glm::dvec3 contribution = emission * T;

				Lo += contribution;
				T *= bxdf * cosTheta / pdf;

				RT_ASSERT(glm::abs(glm::length2(wi) - 1.0) < 1.0e-5);
				RT_ASSERT(glm::abs(glm::length2(wo) - 1.0) < 1.0e-5);
				RT_ASSERT(glm::abs(glm::length2(shadingPoint.Ng) - 1.0) < 1.0e-5);

				if (glm::all(glm::lessThanEqual(T, glm::dvec3(0.0)))) {
					break;
				}

				// ロシアンルーレット
				double max_compornent = glm::compMax(T);
				RT_ASSERT(0.0 <= max_compornent);
				double continue_p = i < 5 ? 1.0 : glm::min(max_compornent, 1.0);
				if (continue_p < random->uniform()) {
					// reject
					break;
				}
				T /= continue_p;
				
				//if (NoI < 0.0) {
				//	printf("");
				//}
				//if (glm::any(glm::isnan(T))) {
				//	printf("");
				//}

				//if (directSampler.canSample()) {
				//	auto sqr = [](double x) {
				//		return x * x;
				//	};
				//	if (is_direct) {
				//		// mis_weight = sqr(pdf_direct) / (sqr(pdf_bxdf) + sqr(pdf_direct));
				//		mis_weight = 2;
				//	}
				//	else {
				//		mis_weight = 0;
				//		// mis_weight = sqr(pdf_bxdf) / (sqr(pdf_bxdf) + sqr(pdf_direct));
				//	}
				//}
				//else {
				//	mis_weight = 1;
				//}

				// バイアスする方向は潜り込むときは逆転する
				// が、必ずしもNoIだけで決めていいかどうか微妙なところがある気がするが・・・
				ro = p + (0.0 < NoI ? shadingPoint.Ng : -shadingPoint.Ng) * kSceneEPS;
				rd = wi;
			}
			else {
				break;
			}
		}
		return Lo;
	}

	inline void serial_for(tbb::blocked_range<int> range, std::function<void(const tbb::blocked_range<int> &)> op) {
		op(range);
	}

	class PTRenderer {
	public:
		PTRenderer(std::shared_ptr<rt::Scene> scene)
			: _scene(scene)
			, _image(scene->camera()->resolution_x, scene->camera()->resolution_y) {
			_badSampleNanCount = 0;
			_badSampleInfCount = 0;
			_badSampleNegativeCount = 0;
			_badSampleFireflyCount = 0;
		}
		void step() {
			_steps++;

#if DEBUG_MODE
			int focusX = 200;
			int focusY = 200;

			for (int y = 0; y < _scene->camera.imageHeight(); ++y) {
				for (int x = 0; x < _scene->camera.imageWidth(); ++x) {
					if (x != focusX || y != focusY) {
						continue;
					}
					PeseudoRandom *random = _image.random(x, y);

					glm::dvec3 o;
					glm::dvec3 d;
					_scene->camera.sampleRay(random, x, y, &o, &d);

					auto r = radiance(*_sceneInterface, o, d, random);
					_image.add(x, y, r);
				}
			}
#else

			auto to = [](houdini_alembic::Vector3f p) {
				return glm::dvec3(p.x, p.y, p.z);
			};
			auto camera = _scene->camera();
			glm::dvec3 object_o =
				to(camera->eye) + to(camera->forward) * (double)camera->focusDistance
				+ to(camera->left) * (double)camera->objectPlaneWidth * 0.5
				+ to(camera->up) * (double)camera->objectPlaneHeight * 0.5;
			glm::dvec3 rVector = to(camera->right) * (double)camera->objectPlaneWidth;
			glm::dvec3 dVector = to(camera->down) * (double)camera->objectPlaneHeight;

			double step_x = 1.0 / _image.width();
			double step_y = 1.0 / _image.height();



			tbb::parallel_for(tbb::blocked_range<int>(0, _image.height()), [&](const tbb::blocked_range<int> &range) {
				// serial_for(tbb::blocked_range<int>(0, _image.height()), [&](const tbb::blocked_range<int> &range) {
				for (int y = range.begin(); y < range.end(); ++y) {
					for (int x = 0; x < _image.width(); ++x) {
						PeseudoRandom *random = _image.random(x, y);
						glm::dvec3 o;
						glm::dvec3 d;

						o = to(camera->eye);

						double u = random->uniform();
						double v = random->uniform();
						glm::dvec3 p_objectPlane =
							object_o
							+ rVector * (step_x * (x + u))
							+ dVector * (step_y * (y + v));

						d = glm::normalize(p_objectPlane - o);
						// _scene->camera.sampleRay(random, x, y, &o, &d);
						auto r = radiance(_scene.get(), o, d, random, x, y);

						for (int i = 0; i < r.length(); ++i) {
							if (glm::isnan(r[i])) {
								_badSampleNanCount++;
								r[i] = 0.0;
							}
							else if (glm::isfinite(r[i]) == false) {
								_badSampleInfCount++;
								r[i] = 0.0;
							}
							else if (r[i] < 0.0) {
								_badSampleNegativeCount++;
								r[i] = 0.0;
							}
							if (10000.0 < r[i]) {
								_badSampleFireflyCount++;
								r[i] = 0.0;
							}
						}
						_image.add(x, y, r);
					}
				}
			});
#endif
		}
		int stepCount() const {
			return _steps;
		}

		int badSampleNanCount() const {
			return _badSampleNanCount.load();
		}
		int badSampleInfCount() const {
			return _badSampleInfCount.load();
		}
		int badSampleNegativeCount() const {
			return _badSampleNegativeCount.load();
		}
		int badSampleFireflyCount() const {
			return _badSampleFireflyCount.load();
		}

		std::shared_ptr<rt::Scene> _scene;
		Image _image;
		int _steps = 0;
		std::atomic<int> _badSampleNanCount;
		std::atomic<int> _badSampleInfCount;
		std::atomic<int> _badSampleNegativeCount;
		std::atomic<int> _badSampleFireflyCount;
	};
}