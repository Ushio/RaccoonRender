#pragma once

#define CATCH_CONFIG_RUNNER
#include "catch.hpp"
#include "ofMain.h"

#include "online.hpp"
#include "peseudo_random.hpp"
#include "orthonormal_basis.hpp"
#include "spherical_sampler.hpp"
#include "triangle_sampler.hpp"
#include "plane_equation.hpp"
#include "triangle_util.hpp"
#include "assertion.hpp"
#include "value_prportional_sampler.hpp"
#include "plot.hpp"

using DefaultRandom = rt::Xoshiro128Plus;

void run_unit_test() {
	static Catch::Session session;
	//char* custom_argv[] = {
	//	"",
	//	"[random]"
	//};
	// session.run(sizeof(custom_argv) / sizeof(custom_argv[0]), custom_argv);
	session.run();
}

TEST_CASE("random", "[random]") {
	auto run = [](rt::PeseudoRandom *random) {
		int k = 5;
		std::vector<int>    hist(k);
		std::vector<double> prob(k);
		int N = 300000;

		rt::GNUPlot4 plot;
		for (int i = 0; i < N; ++i) {
			int index = (int)random->uniform(0.0, k);
			REQUIRE(index <= k);
			index = std::min(index, k - 1);
			hist[index]++;

			for (int j = 0; j < k; ++j) {
				int n = i + 1;
				prob[j] = 100.0 * (double)hist[j] / n;
			}

			//if (1000 < i && i % 100 == 0) {
			//	plot.add((double)i, prob[0], prob[1], prob[2], prob[3]);
			//}
		}
		//plot.show("0", "1", "2", "3");
		//std::cin.get();

		for (int j = 0; j < k; ++j) {
			REQUIRE(prob[j] == Approx(20.0).margin(0.5));
		}
	};
	SECTION("XoroshiroPlus128") {
		run(&rt::XoroshiroPlus128(8));
	}
	SECTION("Xoshiro128Plus") {
		run(&rt::Xoshiro128Plus(3));
	}
	SECTION("PCG") {
		run(&rt::PCG32(43, 1));
	}
	SECTION("MT") {
		run(&rt::MT(6));
	}
}

TEST_CASE("online", "[online]") {
	rt::XoroshiroPlus128 random;
	for (int i = 0; i < 100; ++i) {
		std::vector<double> xs;
		rt::OnlineVariance<double> ov;
		for (int j = 0; j < 100; ++j) {
			double x = random.uniform(0.0, 10.0);
			xs.push_back(x);
			ov.addSample(x);

			double mean;
			double variance;
			rt::mean_and_variance(xs, &mean, &variance);

			REQUIRE(std::abs(mean - ov.mean()) < 1.0e-9);
			REQUIRE(std::abs(variance - ov.variance()) < 1.0e-9);
		}
	}
}

TEST_CASE("sample_on_unit_sphere", "[sample_on_unit_sphere]") {
	rt::XoroshiroPlus128 random;

	SECTION("sample_on_unit_sphere") {
		for (int j = 0; j < 10; ++j)
		{
			glm::dvec3 c;
			int N = 100000;
			for (int i = 0; i < N; ++i) {
				auto sample = rt::sample_on_unit_sphere(&random);
				REQUIRE(glm::length2(sample) == Approx(1.0).margin(1.0e-8));
				c += sample;
			}
			c /= N;
			REQUIRE(std::abs(c.x) < 0.01);
			REQUIRE(std::abs(c.y) < 0.01);
			REQUIRE(std::abs(c.z) < 0.01);
		}
	}

	SECTION("sample_on_unit_hemisphere") {
		for (int j = 0; j < 10; ++j)
		{
			glm::dvec3 c;
			int N = 100000;
			for (int i = 0; i < N; ++i) {
				auto sample = rt::sample_on_unit_hemisphere(&random);
				REQUIRE(glm::length2(sample) == Approx(1.0).margin(1.0e-8));

				if (random.uniform() < 0.5) {
					sample.z = -sample.z;
				}

				c += sample;
			}
			c /= N;
			REQUIRE(std::abs(c.x) < 0.01);
			REQUIRE(std::abs(c.y) < 0.01);
			REQUIRE(std::abs(c.z) < 0.01);
		}
	}
}

TEST_CASE("OrthonormalBasis", "[OrthonormalBasis]") {
	DefaultRandom random;
	for (int j = 0; j < 100000; ++j) {
		auto zAxis = rt::sample_on_unit_sphere(&random);
		rt::OrthonormalBasis space(zAxis);

		REQUIRE(glm::dot(space.xaxis, space.yaxis) == Approx(0.0).margin(1.0e-15));
		REQUIRE(glm::dot(space.yaxis, space.zaxis) == Approx(0.0).margin(1.0e-15));
		REQUIRE(glm::dot(space.zaxis, space.xaxis) == Approx(0.0).margin(1.0e-15));

		glm::dvec3 maybe_zaxis = glm::cross(space.xaxis, space.yaxis);
		for (int j = 0; j < 3; ++j) {
			REQUIRE(glm::abs(space.zaxis[j] - maybe_zaxis[j]) < 1.0e-15);
		}

		auto anyvector = sample_on_unit_sphere(&random);
		auto samevector = space.localToGlobal(space.globalToLocal(anyvector));

		for (int j = 0; j < 3; ++j) {
			REQUIRE(glm::abs(anyvector[j] - samevector[j]) < 1.0e-15);
		}
	}
}

TEST_CASE("PlaneEquation", "[PlaneEquation]") {
	SECTION("basic") {
		DefaultRandom random;
		for (int i = 0; i < 100; ++i) {
			auto n = rt::sample_on_unit_sphere(&random);
			glm::dvec3 point_on_plane = { random.uniform(), random.uniform(), random.uniform() };

			rt::PlaneEquation<double> p;
			p.from_point_and_normal(point_on_plane, n);
			REQUIRE(p.signed_distance(point_on_plane) == Approx(0.0).margin(1.0e-9));

			rt::OrthonormalBasis space(n);
			REQUIRE(p.signed_distance(point_on_plane + space.xaxis) == Approx(0.0).margin(1.0e-9));
			REQUIRE(p.signed_distance(point_on_plane + space.yaxis) == Approx(0.0).margin(1.0e-9));

			for (int j = 0; j < 10; ++j) {
				double d = random.uniform(-5, 5);
				REQUIRE(p.signed_distance(point_on_plane + space.zaxis * d) == Approx(d).margin(1.0e-9));
			}
		}
	}

}

TEST_CASE("triangle_util", "[triangle_util]") {
	SECTION("triangle_normal_cw") {
		DefaultRandom random;

		// xz-plane
		for (int i = 0; i < 1000; ++i) {
			glm::dvec3 p0;
			glm::dvec3 p1 = { random.uniform(), 0.0, random.uniform() };
			glm::dvec3 p2 = { random.uniform(), 0.0, random.uniform() };

			if (glm::angle(p1, p2) < 1.0e-5) {
				continue;
			}

			glm::dvec3 n = rt::triangle_normal_cw(p0, p1, p2);
			REQUIRE(glm::length2(n) == Approx(1.0).margin(1.0e-8));
			REQUIRE(glm::abs(n.x) == Approx(0.0).margin(1.0e-9));
			REQUIRE(glm::abs(n.y) == Approx(1.0).margin(1.0e-9));
			REQUIRE(glm::abs(n.z) == Approx(0.0).margin(1.0e-9));
		}

		// yz-plane
		for (int i = 0; i < 1000; ++i) {
			glm::dvec3 p0;
			glm::dvec3 p1 = { 0.0, random.uniform(), random.uniform() };
			glm::dvec3 p2 = { 0.0, random.uniform(), random.uniform() };

			if (glm::angle(p1, p2) < 1.0e-5) {
				continue;
			}

			glm::dvec3 n = rt::triangle_normal_cw(p0, p1, p2);
			REQUIRE(glm::length2(n) == Approx(1.0).margin(1.0e-8));
			REQUIRE(glm::abs(n.x) == Approx(1.0).margin(1.0e-9));
			REQUIRE(glm::abs(n.y) == Approx(0.0).margin(1.0e-9));
			REQUIRE(glm::abs(n.z) == Approx(0.0).margin(1.0e-9));
		}

		// xy-plane
		for (int i = 0; i < 1000; ++i) {
			glm::dvec3 p0;
			glm::dvec3 p1 = { random.uniform(), random.uniform(), 0.0 };
			glm::dvec3 p2 = { random.uniform(), random.uniform(), 0.0 };

			if (glm::angle(p1, p2) < 1.0e-5) {
				continue;
			}

			glm::dvec3 n = rt::triangle_normal_cw(p0, p1, p2);
			REQUIRE(glm::length2(n) == Approx(1.0).margin(1.0e-8));
			REQUIRE(glm::abs(n.x) == Approx(0.0).margin(1.0e-9));
			REQUIRE(glm::abs(n.y) == Approx(0.0).margin(1.0e-9));
			REQUIRE(glm::abs(n.z) == Approx(1.0).margin(1.0e-9));
		}
	}
}

TEST_CASE("triangle sampler", "[triangle sampler]") {
	DefaultRandom random;
	for (int j = 0; j < 10; ++j) {
		glm::dvec3 center_expect;

		glm::dvec3 p0 = { random.uniform(), random.uniform(), random.uniform() };
		glm::dvec3 p1 = { random.uniform(), random.uniform(), random.uniform() };
		glm::dvec3 p2 = { random.uniform(), random.uniform(), random.uniform() };
		glm::dvec3 c = (p0 + p1 + p2) / 3.0;

		int N = 100000;
		for (int j = 0; j < N; ++j) {
			auto sample = rt::uniform_on_triangle(random.uniform(), random.uniform());
			glm::dvec3 n = rt::triangle_normal_cw(p0, p1, p2);
			rt::PlaneEquation<double> plane;
			plane.from_point_and_normal(p0, n);

			glm::dvec3 s = sample.evaluate(p0, p1, p2);
			REQUIRE(plane.signed_distance(p0) == Approx(0.0).margin(1.0e-8));

			/*
			{sx}   { p_0x, p_1x, p_2x }   {a}
			{sy} = { p_0y, p_1y, p_2y } X {b}
			{sz}   { p_0z, p_1z, p_2z }   {c}

			もし s が三角形の内側なら

			a > 0
			b > 0
			c > 0

			であるはずだ
			*/
			glm::dmat3 m = {
				p0.x, p0.y, p0.z,
				p1.x, p1.y, p1.z,
				p2.x, p2.y, p2.z,
			};
			glm::dvec3 abc = glm::inverse(m) * s;
			REQUIRE(abc.x > 0.0);
			REQUIRE(abc.y > 0.0);
			REQUIRE(abc.z > 0.0);

			center_expect += s;
		}
		center_expect /= N;
		REQUIRE(center_expect.x == Approx(c.x).margin(1.0e-2));
		REQUIRE(center_expect.y == Approx(c.y).margin(1.0e-2));
		REQUIRE(center_expect.z == Approx(c.z).margin(1.0e-2));
	}
}

TEST_CASE("ValueProportionalSampler", "[ValueProportionalSampler]") {
	rt::XoroshiroPlus128 random;

	for (int j = 0; j < 10; ++j)
	{
		rt::ValueProportionalSampler<double> sampler;
		for (int i = 0; i < 5; ++i) {
			sampler.add(random.uniform());
		}

		std::vector<int> h(sampler.size());
		int N = 1000000;
		for (int i = 0; i < N; ++i) {
			h[sampler.sample(&random)]++;
		}
		for (int i = 0; i < h.size(); ++i) {
			double prob = (double)h[i] / N;
			REQUIRE(prob == Approx(sampler.probability(i)).margin(1.0e-2));
		}
	}
}