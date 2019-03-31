#pragma once

#include <vector>
#include <stack>

namespace rt {

	class AliasMethod {
	public:
		void prepare(const std::vector<float> &weights) {
			probs.clear();
			buckets.clear();

			float w_sum = std::accumulate(weights.begin(), weights.end(), 0.0f, [](float a, float b) { return a + b; });

			int N = weights.size();
			probs.resize(N);
			buckets.resize(N);
			for (int i = 0; i < N; ++i) {
				probs[i] = weights[i] / w_sum;
				buckets[i].height = probs[i] * N;
			}

			//float h_sum = 0.0f;
			//for (int i = 0; i < N; ++i) {
			//	h_sum += buckets[i].height;
			//}
			//float h_avg = h_sum / N;

			std::stack<int> lower;
			std::stack<int> upper;

			for (int i = 0; i < N; ++i) {
				if (buckets[i].height < 1.0f) {
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

				assert(1.0f <= buckets[upper_index].height);

				float mov = 1.0f - buckets[lower_index].height;
				buckets[upper_index].height -= mov;
				buckets[lower_index].alias = upper_index;

				if (buckets[upper_index].height < 1.0f) {
					lower.push(upper_index);
				}
				else {
					upper.push(upper_index);
				}

				// lower is already completed
			}
		}

		float probability(int i) const {
			return probs[i];
		}
		int sample(float u0, float u1) const {
			float indexf = u0 * buckets.size();
			int index = (int)(indexf);
			index = std::max(index, 0);
			index = std::min(index, (int)buckets.size() - 1);
			if (buckets[index].alias < 0) {
				return index;
			}
			return u1 < buckets[index].height ? index : buckets[index].alias;
		}

		struct Bucket {
			double height = 0.0f;
			int alias = -1;
		};
		std::vector<double> probs;
		std::vector<Bucket> buckets;
	};
}