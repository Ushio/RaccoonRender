#pragma once
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "orthonormal_basis.hpp"
namespace rt {
	class BxDF;

	class ShadingPoint {
	public:
		double u = 0.0;
		double v = 0.0;
		glm::dvec3 Ng;
		const BxDF *bxdf = nullptr;
	};

	class BxDF {
	public:
		virtual ~BxDF() {}

		// evaluate emission
		virtual glm::dvec3 emission(const glm::dvec3 &wo, const ShadingPoint &shadingPoint) const {
			return glm::dvec3(0.0);
		}

		virtual bool can_direct_sampling() const {
			return true;
		}

		// evaluate bxdf
		virtual glm::dvec3 bxdf(const glm::dvec3 &wo, const glm::dvec3 &wi, const ShadingPoint &shadingPoint) const = 0;

		// sample wi
		virtual glm::dvec3 sample(PeseudoRandom *random, const glm::dvec3 &wo, const ShadingPoint &shadingPoint) const = 0;

		// pdf for wi
		virtual double pdf(const glm::dvec3 &wo, const glm::dvec3 &sampled_wi, const ShadingPoint &shadingPoint) const = 0;
	};

	// p(w) = cosθ / π
	class CosThetaProportionalSampler {
	public:
		static glm::dvec3 sample(PeseudoRandom *random, const glm::dvec3 &Ng) {
			double x;
			double y;
			double r_sqared;
			do {
				x = random->uniform(-1.0, 1.0);
				y = random->uniform(-1.0, 1.0);
				r_sqared = x * x + y * y;
			} while (r_sqared >= 1.0);
			double z = std::sqrt(std::max(1.0 - r_sqared, 0.0));

			OrthonormalBasis basis(Ng);
			return basis.localToGlobal(glm::dvec3(x, y, z));
		}
		static double pdf(const glm::dvec3 &sampled_wi, const glm::dvec3 &Ng) {
			double cosTheta = glm::dot(sampled_wi, Ng);
			if (cosTheta < 0.0) {
				return 0.0;
			}
			return cosTheta * glm::one_over_pi<double>();
		}
	};

	class LambertianBRDF : public BxDF {
	public:
		LambertianBRDF() :Le(0.0), R(1.0) {}
		LambertianBRDF(glm::dvec3 e, glm::dvec3 r, bool back) : Le(e), R(r), backEmission(back) {}
		glm::dvec3 Le;
		glm::dvec3 R;
		bool backEmission = false;

		glm::dvec3 Nv[3];
		bool shadingNormal = false;

		bool isEmission() const {
			return glm::any(glm::greaterThanEqual(Le, glm::dvec3(glm::epsilon<double>())));
		}
		glm::dvec3 emission(const glm::dvec3 &wo, const ShadingPoint &shadingPoint) const override {
			if (backEmission == false && glm::dot(shadingPoint.Ng, wo) < 0.0) {
				return glm::dvec3(0.0);
			}
			return Le;
		}

		glm::dvec3 bxdf(const glm::dvec3 &wo, const glm::dvec3 &wi, const ShadingPoint &shadingPoint) const override {
			// wo, wiは法線に対して同じ向きである必要がある
			// 面から光はリークしない
			if (glm::dot(shadingPoint.Ng, wi) * glm::dot(shadingPoint.Ng, wo) < 0.0) {
				return glm::dvec3(0.0);
			}

			if (shadingNormal) {
				glm::dvec3 Ns = (1.0 - shadingPoint.u - shadingPoint.v) * Nv[0] + shadingPoint.u * Nv[1] + shadingPoint.v * Nv[2];
				Ns = glm::normalize(Ns);
				return glm::abs(glm::dot(Ns, wi) / glm::dot(shadingPoint.Ng, wi)) * glm::dvec3(R) * glm::one_over_pi<double>();
			}

			return glm::dvec3(R) * glm::one_over_pi<double>();
		}
		glm::dvec3 sample(PeseudoRandom *random, const glm::dvec3 &wo, const ShadingPoint &shadingPoint) const override {
			bool isNormalFlipped = glm::dot(wo, shadingPoint.Ng) < 0.0;
			return CosThetaProportionalSampler::sample(random, isNormalFlipped ? -shadingPoint.Ng : shadingPoint.Ng);
		}
		virtual double pdf(const glm::dvec3 &wo, const glm::dvec3 &sampled_wi, const ShadingPoint &shadingPoint) const override {
			bool isNormalFlipped = glm::dot(sampled_wi, shadingPoint.Ng) < 0.0;
			return CosThetaProportionalSampler::pdf(sampled_wi, isNormalFlipped ? -shadingPoint.Ng : shadingPoint.Ng);
		}
	};
}