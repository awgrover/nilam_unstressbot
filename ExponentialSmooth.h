class ExponentialSmooth {
  // a simple formula to sort-of do averaging: exponential smoothing

  const float factor; // forces operations to float space, actually a whole number
  float _smoothed; // need float for converging, i.e. no rounding loss funniness. use "(int) $thisobject", to get value.

  public:

  // "factor" is kind of like the the number of samples that are averaged.
  // So, "5" is sort of like taking 5 samples and averaging them.
  ExponentialSmooth(const int factor) : factor(factor) {};

  int smoothed() { return (int) _smoothed; }
  operator int() const { return (int) _smoothed; }

  void reset(int v) { _smoothed = v; }

  // we intend it to inline
  int average(const int raw_value) { 
    _smoothed = raw_value / factor + _smoothed - _smoothed / factor; 
    /*
    Serial.print("X"); Serial.print((int) this); Serial.print("/");
    Serial.print(factor); Serial.print(" ");
    Serial.print(raw_value); Serial.print(" ");
    Serial.print(_smoothed); Serial.print("|"); Serial.print((int)_smoothed); Serial.print("|"); Serial.print((int) *this);
    Serial.print(" ");
    */

    return (int) _smoothed;
    }

  };
