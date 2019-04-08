﻿#pragma once
#include <algorithm>
#include <random>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

// 基本的な擬似乱数
namespace rt {
	struct PeseudoRandom {
		virtual ~PeseudoRandom() {}

		/* double */

		// 0.0 <= x < 1.0
		virtual double uniform64f() = 0;

		// 0.0 <= x < 1.0
		double uniform() {
			return uniform64f();
		}
		// a <= x < b
		double uniform(double a, double b) {
			return glm::mix(a, b, uniform64f());
		}

		/* float */
		// 0.0 <= x < 1.0
		virtual float uniform32f() {
			return uniform64f();
		}

		// 0.0 <= x < 1.0
		float uniformf() {
			return uniform32f();
		}
		// a <= x < b
		float uniformf(float a, float b) {
			return glm::mix(a, b, uniform32f());
		}
	};

	// http://xoshiro.di.unimi.it/splitmix64.c
	// for generate seed
	struct splitmix {
		uint64_t x = 0; /* The state can be seeded with any value. */
		uint64_t next() {
			uint64_t z = (x += 0x9e3779b97f4a7c15);
			z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
			z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
			return z ^ (z >> 31);
		}
	};

	/*
	http://xoshiro.di.unimi.it/xoroshiro128plus.c
	*/
	struct XoroshiroPlus128 : public PeseudoRandom {
		XoroshiroPlus128() {
			splitmix sp;
			sp.x = 38927482;
			s[0] = std::max(sp.next(), 1ULL);
			s[1] = std::max(sp.next(), 1ULL);
		}
		XoroshiroPlus128(uint64_t seed) {
			splitmix sp;
			sp.x = seed;
			s[0] = std::max(sp.next(), 1ULL);
			s[1] = std::max(sp.next(), 1ULL);
		}

		double uniform64f() override {
			uint64_t x = next();
			uint64_t bits = (0x3FFULL << 52) | (x >> 12);
			return *reinterpret_cast<double *>(&bits) - 1.0;
		}
		float uniform32f() override {
			uint64_t x = next();
			uint32_t bits = ((uint32_t)x >> 9) | 0x3f800000;
			float value = *reinterpret_cast<float *>(&bits) - 1.0f;
			return value;
		}
		/* This is the jump function for the generator. It is equivalent
		to 2^64 calls to next(); it can be used to generate 2^64
		non-overlapping subsequences for parallel computations. */
		void jump() {
			static const uint64_t JUMP[] = { 0xdf900294d8f554a5, 0x170865df4b3201fc };

			uint64_t s0 = 0;
			uint64_t s1 = 0;
			for (int i = 0; i < sizeof JUMP / sizeof *JUMP; i++)
			{
				for (int b = 0; b < 64; b++) {
					if (JUMP[i] & UINT64_C(1) << b) {
						s0 ^= s[0];
						s1 ^= s[1];
					}
					next();
				}
			}

			s[0] = s0;
			s[1] = s1;
		}
	private:
		uint64_t rotl(const uint64_t x, int k) const {
			return (x << k) | (x >> (64 - k));
		}
		uint64_t next() {
			const uint64_t s0 = s[0];
			uint64_t s1 = s[1];
			const uint64_t result = s0 + s1;

			s1 ^= s0;
			s[0] = rotl(s0, 24) ^ s1 ^ (s1 << 16); // a, b
			s[1] = rotl(s1, 37); // c

			return result;
		}
	private:
		uint64_t s[2];
	};

	/*
	http://xoshiro.di.unimi.it/xoshiro128plus.c
	*/
	struct Xoshiro128Plus : public PeseudoRandom {
		Xoshiro128Plus() {
			splitmix sp;
			sp.x = 38927482;
			for (int i = 0; i < 4; ++i) {
				s[i] = std::max((uint32_t)sp.next(), 1u);
			}
		}
		Xoshiro128Plus(uint32_t seed) {
			splitmix sp;
			sp.x = seed;
			for (int i = 0; i < 4; ++i) {
				s[i] = std::max((uint32_t)sp.next(), 1u);
			}
		}

		double uniform64f() override {
			uint64_t a = next() >> 6; // get 26 bit
			uint64_t b = next() >> 6; // get 26 bit
			uint64_t x = a | (b << 26); // 52bit
			uint64_t bits = 0x3FF0000000000000ull | x;
			return *reinterpret_cast<double *>(&bits) - 1.0;
		}
		float uniform32f() override {
			uint32_t x = next();
			uint32_t bits = (x >> 9) | 0x3f800000;
			float value = *reinterpret_cast<float *>(&bits) - 1.0f;
			return value;
		}

		/* 
		This is the jump function for the generator. It is equivalent
		to 2^64 calls to next(); it can be used to generate 2^64
		non-overlapping subsequences for parallel computations. 
		*/
		void jump() {
			static const uint32_t JUMP[] = { 0x8764000b, 0xf542d2d3, 0x6fa035c3, 0x77f2db5b };

			uint32_t s0 = 0;
			uint32_t s1 = 0;
			uint32_t s2 = 0;
			uint32_t s3 = 0;
			for (int i = 0; i < sizeof JUMP / sizeof *JUMP; i++)
				for (int b = 0; b < 32; b++) {
					if (JUMP[i] & UINT32_C(1) << b) {
						s0 ^= s[0];
						s1 ^= s[1];
						s2 ^= s[2];
						s3 ^= s[3];
					}
					next();
				}

			s[0] = s0;
			s[1] = s1;
			s[2] = s2;
			s[3] = s3;
		}
	private:
		inline uint32_t rotl(const uint32_t x, int k) {
			return (x << k) | (x >> (32 - k));
		}
		uint32_t next() {
			const uint32_t result_plus = s[0] + s[3];

			const uint32_t t = s[1] << 9;

			s[2] ^= s[0];
			s[3] ^= s[1];
			s[1] ^= s[2];
			s[0] ^= s[3];

			s[2] ^= t;

			s[3] = rotl(s[3], 11);

			return result_plus;
		}
	private:
		uint32_t s[4];
	};

	struct PCG32 : public PeseudoRandom {
		PCG32() {
			uint64_t initstate = 2;
			uint64_t initseq = 3;

			state = 0ull;
			inc = (initseq << 1u) | 1u;
			next();
			state += initstate;
			next();
		}

		/*
		- initstate is the starting state for the RNG, you can pass any 64-bit value.
		- initseq selects the output sequence for the RNG, you can pass any 64-bit value, although only the low 63 bits are significant.
		*/
		PCG32(uint64_t initstate, uint64_t initseq) {
			state = 0ull;
			inc = (initseq << 1u) | 1u;
			next();
			state += initstate;
			next();
		}

		double uniform64f() override {
			uint64_t a = next() >> 6; // get 26 bit
			uint64_t b = next() >> 6; // get 26 bit
			uint64_t x = a | (b << 26); // 52bit
			uint64_t bits = 0x3FF0000000000000ull | x;
			return *reinterpret_cast<double *>(&bits) - 1.0;
		}
		float uniform32f() override {
			uint32_t x = next();
			uint32_t bits = (x >> 9) | 0x3f800000;
			float value = *reinterpret_cast<float *>(&bits) - 1.0f;
			return value;
		}
	private:
		uint64_t next() {
			uint64_t oldstate = state;
			// Advance internal state
			state = oldstate * 6364136223846793005ULL + (inc | 1);
			// Calculate output function (XSH RR), uses old state for max ILP
			uint32_t xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
			uint32_t rot = oldstate >> 59u;
			return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
		}
	private:
		uint64_t state;
		uint64_t inc;
	};

	struct MT : public PeseudoRandom {
		MT() {

		}
		MT(uint64_t seed) :_engine(seed) {

		}
		double uniform64f() override {
			std::uniform_real_distribution<> d(0.0, 1.0);
			return d(_engine);
		}
		std::mt19937 _engine;
	};
}