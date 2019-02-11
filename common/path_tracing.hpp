#pragma once

#include <tbb/tbb.h>

#include "peseudo_random.hpp"
#include "material.hpp"
#include "scene.hpp"

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

	inline glm::dvec3 radiance(const rt::Scene *scene, glm::dvec3 ro, glm::dvec3 rd, PeseudoRandom *random) {
		// const double kSceneEPS = scene.adaptiveEps();
		const double kSceneEPS = 1.0e-4;
		const double kValueEPS = 1.0e-6;

		glm::dvec3 Lo;
		glm::dvec3 T(1.0);

		bool inside = false;

		constexpr int kDepth = 30;
		for (int i = 0; i < kDepth; ++i) {
			float tmin = 0.0f;
			ShadingPoint shadingPoint;

			glm::dvec3 wo = -rd;

			if (scene->intersect(ro, rd, &shadingPoint, &tmin)) {
				shadingPoint.Ng = glm::normalize(shadingPoint.Ng);

				glm::dvec3 wi = shadingPoint.bxdf->sample(random, wo, shadingPoint);
				glm::dvec3 bxdf = shadingPoint.bxdf->bxdf(wo, wi, shadingPoint);
				glm::dvec3 emission = shadingPoint.bxdf->emission(wo, shadingPoint);
				double pdf = shadingPoint.bxdf->pdf(wo, wi, shadingPoint);
				double NoI = glm::dot(shadingPoint.Ng, wi);
				double cosTheta = std::abs(NoI);

				//if (inside) {
				//	T *= m->beers_law(tmin);
				//}
				//bool over_boundary = NoI < 0.0;
				//if (over_boundary) {
				//	inside = !inside;
				//}

				glm::dvec3 contribution = emission * T;

				Lo += contribution;
				T *= bxdf * cosTheta / pdf;

				//if (has_value(bxdf, kValueEPS)) {
				//	T *= bxdf * cosTheta / pdf;
				//}
				//else {
				//	break;
				//}
				//if (has_value(T, 1.0e-6) == false) {
				//	break;
				//}

				// バイアスする方向は潜り込むときは逆転する
				auto p = ro + rd * (double)tmin;
				ro = p + (0.0 < NoI ? shadingPoint.Ng : -shadingPoint.Ng) * kSceneEPS;
				rd = wi;
			}
			else {
				break;
			}
		}
		return Lo;
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
						auto r = radiance(_scene.get(), o, d, random);

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