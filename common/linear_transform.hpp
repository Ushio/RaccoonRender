#pragma once

namespace rt {
	template <class Real>
	struct LinearTransform {
		LinearTransform(Real a, Real b) :_a(a), _b(b) {}
		LinearTransform(Real inputMin, Real inputMax, Real outputMin, Real outputMax)
		{
			_a = (outputMax - outputMin) / (inputMax - inputMin);
			_b = outputMin - _a * inputMin;
		}
		float evaluate(float x) const {
			return _a * x + _b;
		}
		LinearTransform<Real> inverse() const {
			return LinearTransform(Real(1.0f) / _a, -_b / _a);
		}
	private:
		Real _a;
		Real _b;
	};
}