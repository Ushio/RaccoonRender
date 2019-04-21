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
#include "alias_method.hpp"
#include "n_order_equation.hpp"
#include "plot.hpp"

using DefaultRandom = rt::Xoshiro128StarStar;

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
		std::vector<int>   hist(k);
		std::vector<float> prob(k);
		int N = 300000;

		// rt::GNUPlot4 plot;
		for (int i = 0; i < N; ++i) {
			int index = (int)random->uniform(0.0, k);
			REQUIRE(index <= k);
			index = std::min(index, k - 1);
			hist[index]++;

			for (int j = 0; j < k; ++j) {
				int n = i + 1;
				prob[j] = 100.0f * (float)hist[j] / n;
			}

			//if (1000 < i && i % 100 == 0) {
			//	plot.add((double)i, prob[0], prob[1], prob[2], prob[3]);
			//}
		}
		//plot.ytics(0.5);
		//plot.show("0", "1", "2", "3");
		//std::cin.get();

		for (int j = 0; j < k; ++j) {
			REQUIRE(prob[j] == Approx(20.0f).margin(0.5f));
		}
	};
	SECTION("Xoshiro128StarStar") {
		run(&rt::Xoshiro128StarStar(3));
	}
	SECTION("PCG") {
		run(&rt::PCG32(43, 1));
	}
	SECTION("MT") {
		run(&rt::MT(6));
	}
}

TEST_CASE("online", "[online]") {
	DefaultRandom random;
	for (int i = 0; i < 100; ++i) {
		std::vector<float> xs;
		rt::OnlineVariance<float> ov;
		for (int j = 0; j < 100; ++j) {
			float x = random.uniform(0.0f, 10.0f);
			xs.push_back(x);
			ov.addSample(x);

			float mean;
			float variance;
			rt::mean_and_variance(xs, &mean, &variance);

			REQUIRE(std::abs(mean - ov.mean()) < 1.0e-5f);
			REQUIRE(std::abs(variance - ov.variance()) < 1.0e-5f);
		}
	}
}

TEST_CASE("sample_on_unit_sphere", "[sample_on_unit_sphere]") {
	DefaultRandom random;

	SECTION("sample_on_unit_sphere") {
		for (int j = 0; j < 10; ++j)
		{
			rt::GNUPlot3 plot;

			rt::Kahan<float> cs[3];
			float means[3];
			int N = 1000000;

			for (int i = 0; i < N; ++i) {
				glm::vec3 sample = rt::sample_on_unit_sphere<float>(random.uniform(), random.uniform());
				REQUIRE(glm::length2(sample) == Approx(1.0f).margin(1.0e-5f));

				for(int j = 0; j < 3; ++j) {
					cs[j] += sample[j];
					means[j] = cs[j] / float(i + 1);
				}

				//if (4000 < i && i % 100 == 0) {
				//	plot.add((double)i, means[0], means[1], means[2]);
				//}
			}

			//plot.ytics(0.001);
			//plot.show("x", "y", "z");
			//std::cin.get();

			for (int j = 0; j < 3; ++j) {
				REQUIRE(std::abs(means[j]) < 0.002f);
			}
		}
	}

	SECTION("sample_on_unit_hemisphere") {
		for (int j = 0; j < 10; ++j)
		{
			rt::Kahan<float> cs[3];
			float means[3];
			int N = 1000000;

			for (int i = 0; i < N; ++i) {
				glm::vec3 sample = rt::sample_on_unit_hemisphere<float>(random.uniform(), random.uniform());
				if (random.uniform() < 0.5) {
					sample.z = -sample.z;
				}

				REQUIRE(glm::length2(sample) == Approx(1.0f).margin(1.0e-5f));

				for (int j = 0; j < 3; ++j) {
					cs[j] += sample[j];
					means[j] = cs[j] / float(i + 1);
				}
			}

			for (int j = 0; j < 3; ++j) {
				REQUIRE(std::abs(means[j]) < 0.002f);
			}
		}
	}
}

TEST_CASE("OrthonormalBasis", "[OrthonormalBasis]") {
	DefaultRandom random;
	for (int j = 0; j < 100000; ++j) {
		auto zAxis = rt::sample_on_unit_sphere<double>(random.uniform(), random.uniform());
		rt::OrthonormalBasis<double> space(zAxis);

		REQUIRE(glm::dot(space.xaxis, space.yaxis) == Approx(0.0).margin(1.0e-15));
		REQUIRE(glm::dot(space.yaxis, space.zaxis) == Approx(0.0).margin(1.0e-15));
		REQUIRE(glm::dot(space.zaxis, space.xaxis) == Approx(0.0).margin(1.0e-15));

		glm::dvec3 maybe_zaxis = glm::cross(space.xaxis, space.yaxis);
		for (int j = 0; j < 3; ++j) {
			REQUIRE(glm::abs(space.zaxis[j] - maybe_zaxis[j]) < 1.0e-15);
		}

		auto anyvector = rt::sample_on_unit_sphere<double>(random.uniform(), random.uniform());
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
			auto n = rt::sample_on_unit_sphere<double>(random.uniform(), random.uniform());
			glm::dvec3 point_on_plane = { random.uniform(), random.uniform(), random.uniform() };

			rt::PlaneEquation<double> p;
			p.from_point_and_normal(point_on_plane, n);
			REQUIRE(p.signed_distance(point_on_plane) == Approx(0.0).margin(1.0e-9));

			rt::OrthonormalBasis<double> space(n);
			REQUIRE(p.signed_distance(point_on_plane + space.xaxis) == Approx(0.0).margin(1.0e-9));
			REQUIRE(p.signed_distance(point_on_plane + space.yaxis) == Approx(0.0).margin(1.0e-9));

			for (int j = 0; j < 10; ++j) {
				double d = random.uniform(-5.0f, 5.0f);
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
		glm::vec3 center_expect;

		glm::vec3 p0 = { random.uniform(), random.uniform(), random.uniform() };
		glm::vec3 p1 = { random.uniform(), random.uniform(), random.uniform() };
		glm::vec3 p2 = { random.uniform(), random.uniform(), random.uniform() };
		glm::vec3 c = (p0 + p1 + p2) / 3.0f;

		int N = 100000;
		for (int j = 0; j < N; ++j) {
			auto sample = rt::uniform_on_triangle(random.uniform(), random.uniform());
			glm::vec3 n = rt::triangle_normal_cw(p0, p1, p2);
			rt::PlaneEquation<float> plane;
			plane.from_point_and_normal(p0, n);

			glm::dvec3 s = sample.evaluate(p0, p1, p2);
			REQUIRE(plane.signed_distance(p0) == Approx(0.0f).margin(1.0e-6f));

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
			glm::mat3 m = {
				p0.x, p0.y, p0.z,
				p1.x, p1.y, p1.z,
				p2.x, p2.y, p2.z,
			};
			glm::vec3 abc = glm::inverse(m) * s;
			REQUIRE(abc.x >= -1.0e-5f);
			REQUIRE(abc.y >= -1.0e-5f);
			REQUIRE(abc.z >= -1.0e-5f);

			center_expect += s;
		}
		center_expect /= N;
		REQUIRE(center_expect.x == Approx(c.x).margin(1.0e-2));
		REQUIRE(center_expect.y == Approx(c.y).margin(1.0e-2));
		REQUIRE(center_expect.z == Approx(c.z).margin(1.0e-2));
	}
}

TEST_CASE("ValueProportionalSampler", "[ValueProportionalSampler]") {
	DefaultRandom random;

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

TEST_CASE("AliasMethod", "[AliasMethod]") {
	DefaultRandom random;

	for (int j = 0; j < 10; ++j)
	{
		int buckets = 5;
		std::vector<float> ws;
		for (int i = 0; i < buckets; ++i) {
			ws.push_back(glm::mix(0.1, 1.0, random.uniform()));
		}
		rt::AliasMethod<float> alias;
		alias.prepare(ws);

		// rt::GNUPlot4 plot;

		std::vector<int>    hist(buckets);
		std::vector<float> prob(buckets);
		int N = 10000000;
		for (int i = 0; i < N; ++i) {
			hist[alias.sample(random.uniform_integer(), random.uniform())]++;

			for (int j = 0; j < buckets; ++j) {
				int n = i + 1;
				prob[j] = 100.0f * (float)hist[j] / n;
			}

			//if (2000 < i && i % 100 == 0) {
			//	plot.add((double)i,
			//		std::fabs(prob[0] - alias.probability(0) * 100.0f),
			//		std::fabs(prob[1] - alias.probability(1) * 100.0f),
			//		std::fabs(prob[2] - alias.probability(2) * 100.0f),
			//		std::fabs(prob[3] - alias.probability(3) * 100.0f));
			//}
		}
		//plot.ytics(0.1);
		//plot.show("0", "1", "2", "3");
		//std::cin.get();

		for (int i = 0; i < buckets; ++i) {
			REQUIRE(prob[i] == Approx(alias.probability(i) * 100.0f).margin(0.1f));
		}
	}
}

