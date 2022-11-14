using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace TrackingSmoothing {
	class LowPassFilter {
		float y, a, s;
		bool initialized;

		public void setAlpha(float _alpha) {
			if (_alpha <= 0.0f || _alpha > 1.0f) {
				Console.WriteLine("alpha should be in (0.0., 1.0]");
				return;
			}
			a = _alpha;
		}

		public LowPassFilter(float _alpha, float _initval = 0.0f) {
			y = s = _initval;
			setAlpha(_alpha);
			initialized = false;
		}

		public float Filter(float _value) {
			float result;
			if (initialized)
				result = a * _value + (1.0f - a) * s;
			else {
				result = _value;
				initialized = true;
			}
			y = _value;
			s = result;
			return result;
		}

		public float filterWithAlpha(float _value, float _alpha) {
			setAlpha(_alpha);
			return Filter(_value);
		}

		public bool hasLastRawValue() {
			return initialized;
		}

		public float lastRawValue() {
			return y;
		}

	};

	// -----------------------------------------------------------------

	public class OneEuroFilter {
		float freq;
		float mincutoff;
		float beta;
		float dcutoff;
		LowPassFilter x;
		LowPassFilter dx;
		float lasttime;

		// currValue contains the latest value which have been succesfully filtered
		// prevValue contains the previous filtered value
		public float currValue { get; protected set; }
		public float prevValue { get; protected set; }

		float alpha(float _cutoff) {
			float te = 1.0f / freq;
			float tau = 1.0f / (2.0f * (float)Math.PI * _cutoff);
			return 1.0f / (1.0f + tau / te);
		}

		void setFrequency(float _f) {
			if (_f <= 0.0f) {
				Console.WriteLine("freq should be > 0");
				return;
			}
			freq = _f;
		}

		void setMinCutoff(float _mc) {
			if (_mc <= 0.0f) {
				Console.WriteLine("mincutoff should be > 0");
				return;
			}
			mincutoff = _mc;
		}

		void setBeta(float _b) {
			beta = _b;
		}

		void setDerivateCutoff(float _dc) {
			if (_dc <= 0.0f) {
				Console.WriteLine("dcutoff should be > 0");
				return;
			}
			dcutoff = _dc;
		}

		public OneEuroFilter(float _freq, float _mincutoff = 1.0f, float _beta = 0.0f, float _dcutoff = 1.0f) {
			setFrequency(_freq);
			setMinCutoff(_mincutoff);
			setBeta(_beta);
			setDerivateCutoff(_dcutoff);
			x = new LowPassFilter(alpha(mincutoff));
			dx = new LowPassFilter(alpha(dcutoff));
			lasttime = -1.0f;

			currValue = 0.0f;
			prevValue = currValue;
		}

		public void UpdateParams(float _freq, float _mincutoff = 1.0f, float _beta = 0.0f, float _dcutoff = 1.0f) {
			setFrequency(_freq);
			setMinCutoff(_mincutoff);
			setBeta(_beta);
			setDerivateCutoff(_dcutoff);
			x.setAlpha(alpha(mincutoff));
			dx.setAlpha(alpha(dcutoff));
		}

		public float Filter(float value, float timestamp = -1.0f) {
			prevValue = currValue;

			// update the sampling frequency based on timestamps
			if (lasttime != -1.0f && timestamp != -1.0f)
				freq = 1.0f / (timestamp - lasttime);
			lasttime = timestamp;
			// estimate the current variation per second 
			float dvalue = x.hasLastRawValue() ? (value - x.lastRawValue()) * freq : 0.0f; // FIXME: 0.0 or value? 
			float edvalue = dx.filterWithAlpha(dvalue, alpha(dcutoff));
			// use it to update the cutoff frequency
			float cutoff = mincutoff + beta * (float)Math.Abs(edvalue);
			// filter the given value
			currValue = x.filterWithAlpha(value, alpha(cutoff));

			return currValue;
		}
	};


	// this class instantiates an array of OneEuroFilter objects to filter each component of Vector2, Vector3, Vector4 or Quaternion types
	public class OneEuroFilter<T> where T : struct {
		// containst the type of T
		Type type;
		// the array of filters
		OneEuroFilter[] oneEuroFilters;

		// filter parameters
		public float freq { get; protected set; }
		public float mincutoff { get; protected set; }
		public float beta { get; protected set; }
		public float dcutoff { get; protected set; }

		// currValue contains the latest value which have been succesfully filtered
		// prevValue contains the previous filtered value
		public T currValue { get; protected set; }
		public T prevValue { get; protected set; }

		// initialization of our filter(s)
		public OneEuroFilter(float _freq, float _mincutoff = 1.0f, float _beta = 0.0f, float _dcutoff = 1.0f) {
			type = typeof(T);
			currValue = new T();
			prevValue = new T();

			freq = _freq;
			mincutoff = _mincutoff;
			beta = _beta;
			dcutoff = _dcutoff;

			if (type == typeof(Vector2))
				oneEuroFilters = new OneEuroFilter[2];

			else if (type == typeof(Vector3))
				oneEuroFilters = new OneEuroFilter[3];

			else if (type == typeof(Vector4) || type == typeof(Quaternion))
				oneEuroFilters = new OneEuroFilter[4];
			else {
				Console.WriteLine(type + " is not a supported type");
				return;
			}

			for (int i = 0; i < oneEuroFilters.Length; i++)
				oneEuroFilters[i] = new OneEuroFilter(freq, mincutoff, beta, dcutoff);
		}

		// updates the filter parameters
		public void UpdateParams(float _freq, float _mincutoff = 1.0f, float _beta = 0.0f, float _dcutoff = 1.0f) {
			freq = _freq;
			mincutoff = _mincutoff;
			beta = _beta;
			dcutoff = _dcutoff;

			for (int i = 0; i < oneEuroFilters.Length; i++)
				oneEuroFilters[i].UpdateParams(freq, mincutoff, beta, dcutoff);
		}


		// filters the provided _value and returns the result.
		// Note: a timestamp can also be provided - will override filter frequency.
		public T Filter<U>(U _value, float timestamp = -1.0f) where U : struct {
			prevValue = currValue;

			if (typeof(U) != type) {
				Console.WriteLine("WARNING! " + typeof(U) + " when " + type + " is expected!\nReturning previous filtered value");
				currValue = prevValue;

				return (T)Convert.ChangeType(currValue, typeof(T));
			}

			if (type == typeof(Vector3)) {
				Vector3 output = Vector3.Zero;
				Vector3 input = (Vector3)Convert.ChangeType(_value, typeof(Vector3));

				output.X = oneEuroFilters[0].Filter(input.X, timestamp);
				output.Y = oneEuroFilters[1].Filter(input.Y, timestamp);
				output.Z = oneEuroFilters[2].Filter(input.Z, timestamp);

				currValue = (T)Convert.ChangeType(output, typeof(T));
			} else if (type == typeof(Quaternion)) {
				
				Quaternion output = Quaternion.Identity;
				Quaternion input = (Quaternion)Convert.ChangeType(_value, typeof(Quaternion));

				output.X = oneEuroFilters[0].Filter(input.X, timestamp);
				output.Y = oneEuroFilters[1].Filter(input.Y, timestamp);
				output.Z = oneEuroFilters[2].Filter(input.Z, timestamp);
				output.W = oneEuroFilters[3].Filter(input.W, timestamp);
				output = Quaternion.Normalize(output);

				currValue = (T)Convert.ChangeType(output, typeof(T));
			}

			return (T)Convert.ChangeType(currValue, typeof(T));
		}
	}
}
