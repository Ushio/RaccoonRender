#pragma once

#include <vector>
#include <stack>

namespace rt {
	template <class Real>
	class Kahan {
	public:
		Kahan() {}
		Kahan(Real value) {}

		void add(Real x) {
			auto y = x - _c;
			auto t = _sum + y;
			_c = (t - _sum) - y;
			_sum = t;
		}
		void sub(Real x) {
			add(-x);
		}
		void operator=(Real x) {
			_sum = x;
			_c = Real(0.0);
		}
		void operator+=(Real x) {
			add(x);
		}
		void operator-=(Real x) {
			sub(x);
		}
		operator Real() const {
			return _sum - _c;
		}
		Real get() const {
			return *this;
		}
	private:
		Real _sum = Real(0.0);
		Real _c = Real(0.0);
	};
	template <class Real>
	class AliasMethod {
	public:
		void prepare(const std::vector<Real> &weights) {
			probs.clear();
			buckets.clear();

			Kahan<Real> w_sum;
			for (auto v : weights) {
				w_sum += v;
			}
			Real one_over_weight_sum = Real(1.0) / w_sum.get();

			int N = weights.size();
			probs.resize(N);
			buckets.resize(N);
			for (int i = 0; i < N; ++i) {
				probs[i] = weights[i] * one_over_weight_sum;
				buckets[i].height = probs[i] * N;
			}

			//Kahan<Real> h_sum;
			//for (int i = 0; i < N; ++i) {
			//	h_sum += buckets[i].height;
			//}
			//Real h_avg = h_sum / N;

			std::stack<int> lower;
			std::stack<int> upper;

			for (int i = 0; i < N; ++i) {
				if (buckets[i].height < Real(1.0)) {
					lower.push(i);
				}
				else {
					upper.push(i);
				}
			}

			for (;;) {
				if (lower.empty() || upper.empty()) {
					break;
				}

				int lower_index = lower.top();
				lower.pop();

				int upper_index = upper.top();
				upper.pop();

				assert(Real(1.0) <= buckets[upper_index].height);

				Real mov = Real(1.0) - buckets[lower_index].height;
				buckets[upper_index].height -= mov;
				buckets[lower_index].alias = upper_index;

				if (buckets[upper_index].height < Real(1.0)) {
					lower.push(upper_index);
				}
				else {
					upper.push(upper_index);
				}

				// lower is already completed
			}
		}

		Real probability(int i) const {
			return probs[i];
		}
		int sample(Real u0, Real u1) const {
			Real indexf = u0 * buckets.size();
			int index = (int)(indexf);
			index = std::max(index, 0);
			index = std::min(index, (int)buckets.size() - 1);
			if (buckets[index].alias < 0) {
				return index;
			}
			return u1 < buckets[index].height ? index : buckets[index].alias;
		}

		struct Bucket {
			Kahan<Real> height = Real(0.0);
			int alias = -1;
		};
		std::vector<Real> probs;
		std::vector<Bucket> buckets;
	};
}