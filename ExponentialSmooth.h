class ExponentialSmooth {
  // a simple formula to sort-of do averaging: exponential smoothing

  float smoothed; // need float for converging, i.e. no rounding loss funniness. use "(int) $thisobject", to get value.
  const float factor; // forces operations to float space, actually a whole number

  public:

  // "factor" is kind of like the the number of samples that are averaged.
  // So, "5" is sort of like taking 5 samples and averaging them.
  ExponentialSmooth(const int factor) : factor(factor) {};

  operator int() const {return (int) smoothed;} // automatically let us be used as an int
  operator unsigned long() const {return (unsigned long) smoothed;} // automatically let us be used as unsigned long

  // we intend it to inline
  int average(const int raw_value) { 
    smoothed = raw_value / factor + smoothed - smoothed / factor; 
    return (int) smoothed;
    }

  };
