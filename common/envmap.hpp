#pragma once
#include <glm/glm.hpp>
#include "alias_method.hpp"

namespace rt {

	class EnvironmentMap {
	public:
		virtual ~EnvironmentMap() {}
		virtual glm::dvec3 radiance(const glm::dvec3 &wi) const = 0;
		virtual float pdf(const glm::dvec3 &rd, const glm::dvec3 &n) const = 0;
		virtual glm::dvec3 sample(PeseudoRandom *random, const glm::dvec3 &n) const = 0;
	};

	class ConstantEnvmap : public EnvironmentMap {
	public:
		virtual glm::dvec3 radiance(const glm::dvec3 &wi) const {
			return constant;
		}
		virtual float pdf(const glm::dvec3 &rd, const glm::dvec3 &n) const override {
			return CosThetaProportionalSampler::pdf(rd, n);
		}
		virtual glm::dvec3 sample(PeseudoRandom *random, const glm::dvec3 &n) const override {
			return CosThetaProportionalSampler::sample(random, n);
		}
		glm::dvec3 constant;
	};

	class ImageEnvmap : public EnvironmentMap {
	public:
		ImageEnvmap(std::string filePath) :_texture(new Image2D()) {
			_texture->load(filePath.c_str());

			// テスト用 クランプ
			// _texture->clamp_rgb(0.0f, 20.0f);
			double theta_step = glm::pi<double>() / _texture->height();

			// beg_theta ~ end_thetaの挟まれた領域
			auto solid_angle_sliced_sphere = [](double beg_theta, double end_theta) {
				double beg_y = std::cos(beg_theta);
				double end_y = std::cos(end_theta);
				return (beg_y - end_y) * glm::two_pi<double>();
			};

			// Selection Weight
			const Image2D &image = *_texture;
			std::vector<double> weights(image.width() * image.height());
			for (int y = 0; y < image.height(); ++y) {
				double beg_theta = theta_step * y;
				double end_theta = beg_theta + theta_step;
				double sr = solid_angle_sliced_sphere(beg_theta, end_theta) / _texture->width();

				for (int x = 0; x < image.width(); ++x) {
					auto radiance = image(x, y);
					double Y = 0.2126 * radiance.x + 0.7152 * radiance.y + 0.0722 * radiance.z;

					weights[y * image.width() + x] = Y * sr;
				}
			}
			_aliasMethod.prepare(weights);

			// Precomputed PDF
			_pdf.resize(image.width() * image.height());
			for (int iy = 0; iy < image.height(); ++iy) {
				double beg_theta = theta_step * iy;
				double end_theta = beg_theta + theta_step;
				double sr = solid_angle_sliced_sphere(beg_theta, end_theta) / _texture->width();

				for (int ix = 0; ix < image.width(); ++ix) {
					int index = iy * image.width() + ix;
					double p = _aliasMethod.probability(index);
					_pdf[index] = p * (1.0 / sr);
				}
			}
		}

		// always positive
		// phi = 0.0 ~ 2.0 * pi
		// theta = 0.0 ~ pi
		bool spherical_coordinate_positive(glm::vec3 rd, float *theta, float *phi) const {
			float z = rd.y;
			float x = rd.z;
			float y = rd.x;
			*theta = std::acos(z);
			*phi = std::atan2(y, x);
			if (*phi < 0.0f) {
				*phi += glm::two_pi<float>();
			}
			if (isfinite(*theta) == false || isfinite(*phi) == false) {
				return false;
			}
			return true;
		}

		virtual glm::dvec3 radiance(const glm::dvec3 &rd) const override {
			float theta;
			float phi;
			if (spherical_coordinate_positive(rd, &theta, &phi) == false) {
				return glm::dvec3(0.0);
			}

			RT_ASSERT(0.0 <= phi && phi <= glm::two_pi<float>());

			// 1.0f - is clockwise order envmap
			float u = 1.0f - phi / (2.0f * glm::pi<float>());
			float v = theta / glm::pi<float>();

			// 1.0f - is texture coordinate problem
			return _texture->sample_repeat(u, 1.0f - v);
		}
		float pdf(const glm::dvec3 &rd, const glm::dvec3 &n) const {
			float theta;
			float phi;
			if (spherical_coordinate_positive(rd, &theta, &phi) == false) {
				return 0.0f;
			}

			// 1.0f - is clockwise order envmap
			float u = 1.0f - phi / (2.0f * glm::pi<float>());
			float v = theta / glm::pi<float>();

			int ix = u * _texture->width();
			int iy = v * _texture->height();
			ix = glm::clamp(ix, 0, _texture->width() - 1);
			iy = glm::clamp(iy, 0, _texture->height() - 1);

			return _pdf[iy * _texture->width() + ix];
		}
		virtual glm::dvec3 sample(PeseudoRandom *random, const glm::dvec3 &n) const override {
			const Image2D &image = *_texture;

			int index = _aliasMethod.sample(random->uniform32f(), random->uniform32f());
			int ix = index % image.width();
			int iy = index / image.width();
			float sample_x = ix + random->uniform32f();

			float theta_step = glm::pi<float>() / image.height();
			float beg_theta = theta_step * iy;
			float end_theta = beg_theta + theta_step;
			float beg_y = cos(beg_theta);
			float end_y = cos(end_theta);

			float phi = -glm::two_pi<float>() * (sample_x / image.width());

			float y = glm::mix(beg_y, end_y, random->uniform32f());
			float r_xz = std::sqrt(std::max(1.0f - y * y, 0.0f));

			float x = r_xz * sin(phi);
			float z = r_xz * cos(phi);
			return glm::dvec3(x, y, z);
		}
	private:
		std::unique_ptr<Image2D> _texture;
		std::vector<float> _pdf;
		AliasMethod<double> _aliasMethod;
	};
}