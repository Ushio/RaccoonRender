#pragma once
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <rttr/registration>

#include "peseudo_random.hpp"
#include "orthonormal_basis.hpp"
#include "assertion.hpp"

namespace rt {
	class BxDF;

	class ShadingPoint {
	public:
		double u = 0.0;
		double v = 0.0;
		glm::dvec3 Ng;
		const BxDF *bxdf = nullptr;
	};

	enum class GeoScope : uint8_t {
		Points,
		Vertices,
		Primitives,
	};
	static const std::string kGeoScopeKey = "GeoScope";

	class BxDF {
	public:
		virtual ~BxDF() {}

		virtual BxDF *allocate() const = 0;

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
		LambertianBRDF(glm::dvec3 e, glm::dvec3 r, bool back) : Le(e), R(r), BackEmission(back) {}

		glm::dvec3 Le;
		glm::dvec3 R;
		int BackEmission = 0;
		std::array<glm::dvec3, 3> Nv;
		int ShadingNormal = 0;

		BxDF *allocate() const override {
			return new LambertianBRDF(*this);
		}

		glm::dvec3 emission(const glm::dvec3 &wo, const ShadingPoint &shadingPoint) const override {
			if (BackEmission == 0 && glm::dot(shadingPoint.Ng, wo) < 0.0) {
				return glm::dvec3(0.0);
			}
			return Le;
		}

		glm::dvec3 bxdf(const glm::dvec3 &wo, const glm::dvec3 &wi, const ShadingPoint &shadingPoint) const override {
			// wo, wiは面をまたぐ場合の寄与は0
			if (glm::dot(shadingPoint.Ng, wi) * glm::dot(shadingPoint.Ng, wo) < 0.0) {
				return glm::dvec3(0.0);
			}

			if (ShadingNormal) {
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
			// wo, wiは面をまたぐ場合の確率密度は0
			if (glm::dot(shadingPoint.Ng, sampled_wi) * glm::dot(shadingPoint.Ng, wo) < 0.0) {
				return 0.0;
			}
			bool isNormalFlipped = glm::dot(sampled_wi, shadingPoint.Ng) < 0.0;
			auto p = CosThetaProportionalSampler::pdf(sampled_wi, isNormalFlipped ? -shadingPoint.Ng : shadingPoint.Ng);
			RT_ASSERT(0.0 <= p);
			return p;
		}
	};
	RTTR_REGISTRATION
	{
		using namespace rttr;

		registration::class_<LambertianBRDF>("Lambertian")
		.constructor<>()
		.method("allocate", &LambertianBRDF::allocate)
		.property("Le", &LambertianBRDF::Le)(metadata(kGeoScopeKey, GeoScope::Primitives))
		.property("Cd", &LambertianBRDF::R)(metadata(kGeoScopeKey, GeoScope::Primitives))
		.property("BackEmission", &LambertianBRDF::BackEmission)(metadata(kGeoScopeKey, GeoScope::Primitives))
		.property("N", &LambertianBRDF::Nv)(metadata(kGeoScopeKey, GeoScope::Vertices))
		.property("ShadingNormal", &LambertianBRDF::ShadingNormal)(metadata(kGeoScopeKey, GeoScope::Primitives));
	}

	class WardBRDF : public BxDF {
	public:
		WardBRDF() {}
		
		glm::dvec3 tangentu;
		glm::dvec3 tangentv;

		glm::dvec3 bxdf(const glm::dvec3 &wo, const glm::dvec3 &wi, const ShadingPoint &shadingPoint) const override {
			// wo, wiは面をまたぐ場合の寄与は0
			double NoI = glm::dot(shadingPoint.Ng, wi);
			double NoO = glm::dot(shadingPoint.Ng, wo);
			if (NoI * NoO < 0.0) {
				return glm::dvec3(0.0);
			}
			
			// we can use unnormalized half vector
			glm::dvec3 h = glm::normalize(wo + wi);

			double rho_s = 1.0;
			double ax = 0.8;
			double ay = 0.1;
			//double ax = 0.1;
			//double ay = 0.8;
			//double ax = 0.1;
			//double ay = 0.1;

			auto Ng = NoI < 0.0 ? -shadingPoint.Ng : shadingPoint.Ng;

			auto sqr = [](double x) {
				return x * x;
			};

			//auto brdf = rho_s / (4.0 * glm::pi<double>() * ax * ay * std::sqrt(NoI * NoO)) * glm::exp(
			//	- (sqr(glm::dot(h, tangentu) / ax) + sqr(glm::dot(h, tangentv) / ay)) / sqr(glm::dot(h, Ng))
			//);
			auto brdf = rho_s / (4.0 * glm::pi<double>() * ax * ay * NoI * NoO) * glm::exp(
				-(sqr(glm::dot(h, tangentu) / ax) + sqr(glm::dot(h, tangentv) / ay)) / sqr(glm::dot(h, Ng))
			);
			return glm::dvec3(brdf);
		}
		glm::dvec3 sample(PeseudoRandom *random, const glm::dvec3 &wo, const ShadingPoint &shadingPoint) const override {
			bool isNormalFlipped = glm::dot(wo, shadingPoint.Ng) < 0.0;
			return CosThetaProportionalSampler::sample(random, isNormalFlipped ? -shadingPoint.Ng : shadingPoint.Ng);
		}
		virtual double pdf(const glm::dvec3 &wo, const glm::dvec3 &sampled_wi, const ShadingPoint &shadingPoint) const override {
			// wo, wiは面をまたぐ場合の確率密度は0
			if (glm::dot(shadingPoint.Ng, sampled_wi) * glm::dot(shadingPoint.Ng, wo) < 0.0) {
				return 0.0;
			}
			bool isNormalFlipped = glm::dot(sampled_wi, shadingPoint.Ng) < 0.0;
			auto p = CosThetaProportionalSampler::pdf(sampled_wi, isNormalFlipped ? -shadingPoint.Ng : shadingPoint.Ng);
			RT_ASSERT(0.0 <= p);
			return p;
		}
	};
}