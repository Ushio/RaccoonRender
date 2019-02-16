#pragma once
#include <glm/glm.hpp>
#include <glm/ext.hpp>

namespace rt {
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

		bool intersect_ray(const glm::dvec3 &ro, const glm::dvec3 &rd, double *tmin) const {
			double eps = 1.0e-5f;
			auto denom = glm::dot(n, rd);
			if (std::fabs(denom) < eps) {
				return false;
			}
			auto this_tmin = -(glm::dot(n, ro) + d) / denom;
			if (this_tmin < 0.0) {
				return false;
			}
			*tmin = this_tmin;
			return true;
		}
	};
}