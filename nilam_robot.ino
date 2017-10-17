/*
  Code for Nilam Sari.

  Everything uses the pulse sensor beats-per-minute, bpm.

  * LCD shows different images based on the bpm
  * MP3's are played when the bpm changes
  * Breathing is based on bpm
  * Robot-heart beats based on bpm

*/

// Pulse sensor
const int PulseSensorPin = A0;
const int LED_DuringBeat = 13; // turn on this LED whenever the pulse-value > threshold. ie. _during_ beat
const int Threshold = 512; // 
int BPM_Direction = 0; // positive when BPM is going up, negative when going down, magnitude is how fast the change is. 
const int BeatAveraging = 5; // adjust for stability/smoothing. for base BPM
ExponentialSmooth BPM(BeatAveraging); // can use as an int
// Need min/max for breathing mapping, and our heart beat
const int MinBPM = 40; // lowest reasonable
const int MaxBPM = 180; // highest reasonable
// Smoothing/BPM derivatives
// Make smoothers for things that need them, put in the list for updating
ExponentialSmooth BPM_SmoothLong(BeatAveraging * 3); // for sort-of first-derivate. adjust for "inertia"
const int EyePersistance = BeatAveraging * 4; // smooth over ~4, need responsive, but not change too often
ExponentialSmooth BPM_ForEyes( BeatAveraging * 4 ); // smooth over ~4, need responsive, but not change too often

// Everything in this list will be updated on each beat
ExponentialSmooth BPM_Smoothers[] = { BPM, BPM_ForEyes };

// Breathing just maps BPM to breathing
const int MotorPin = A1; // pwm control for a cam motor
const int BreathToRPM = 100; // multiply the breathingrate to get the pwm value. adjust to motor.
const int MinBreathe = 15; // lowest breathing rate we want to do: correspond to MinBPM
const int MaxBreathe = 80; // highest breathing rate we want to do: correspond to MaxBPM
const int CurrentBreathingRate = MinBreathe; // got to start somewhere

// our heart just maps BPM to rate
const int MinOurBeat = 40; // lowest beat rate we want to do: correspond to MinBPM
const int MaxOurBeat = 80; // highest beat rate we want to do: correspond to MaxBPM
const int CurrentBeatRate = MinBeat; // got to start somewhere
const structure BeatExemplar {
  int hardDuration; // time it takes to drive solenoid fully out: "beat". and any hold time
  int restingDuration; // time between beats.
    // total should be 60 seconds
    // expandable to "ba-dump" eventually
  };
BeatExemplar Beat = { 1, 59 };

// Eyes
// In order of the eye pictures ("eye_1", "eye_2"...):
// Give the corresponding maximum BPM. e.g. { 50, 100 } means: use "eye_1" up through 50 bpm, then "eye_2" up through 100 bpm
// Probably have 0..min, "absurdly low" and "absurdly high" values
// first BPM WILL happen when the pulse-sensor is first attached!
int EyeBins[] = { 40, 50, 60, 80, 120, 140, 200 }; 

void setup() {
  Serial.begin(9600);
}

void loop() {
  processPulse(); // let's process it as often as we can, each time through the loop
  setEyes();
  setBreathingRate();
  runBreathing();
  setOurHeart();
  runOurHeart();
  }

  void runOurHeart() {
    // the heart is a solenoid
    // just do a "beat"
    enum States = { resting, start_beat_a, hard_beat, hard_relax }; // what a beat looks like
    static States state = hard_relax; // start relaxed

    // FIXME: fix units of all bpms to be bpm

    static int hardBeatDuration, restingDuration; //have to convert "per-minute" to durations

    unsigned long now = millis();

    switch(state) {

      resting : 
        if (now > end_state) {
          state = start_beat_a;
          }
        break;

      start_beat_a :
        analogWrite( HeartPin, HardBeat);
        end_state = now + hardBeatDuration;
        state = hard_beat;
        break;

      hard_beat :
        if (now > end_state) {
          state = hard_relax;
          }
        break;

      hard_relax : 
        state = resting;
        end_state = now + restingDuration;

        // let's only recalculate the durations during a relax
        // CurrentBeatRate in per-minute
        // hardbeat..relax = 1 cycle
        // so scale the exemplar
        hardBeatDuration = 1.0/CurrentBeatRate * Beat.hardDuration:
        restingDuration = 1.0/CurrentBeatRate * Beat.restingDuration;
        break;
      
      }

  }

void runBreathing() {
  // breathing is a cam on a motor, so PWM driven
  // so, smoothly change speed from current to target
  // may need to update every loop
  static ExponentialSmooth motor_speed( 10 ); // the factor is how many samples to reach target: how smooth the change is

  analogWrite(MotorPin, motor_speed.average( CurrentBreathingRate ); // pretty simple
  }

void setEyes() {
  // use the BPM_ForEyes, pick an eye picture and display
  static int last_picture = -1; // impossible "last picture" for startup
  
  // We may need to rate limit...

  int which_picture = 0;
  for( int &binmax : EyeBins ) {
    if (BPM_ForEyes > binmax) {
      break; // we found the which_picture last time!
      }
    which_picture = &binmax - &EyeBins[0]; // it's at least this bin
    }
  // if we "fall" out (without using break), which_picture is the last one

  if (which_picture != last_picture) {
    last_picture = which_picture;
    updateEyes( which_picture );
    }

  }

void updateEyes( int picture_index ) {
  // FIXME: pick "eye_$i", copy to lcd
  // Do it in an interrupt routine...
  Serial.print("Picture");
  Serial.println(picture_index);
  }

void setBreathingRate() {
  // not much to do, runBreathing() does the actual breathing
  CurrentBreathingRate = map(BPM, MinBPM, MaxBPM, BreathToRPM * MinBreathe, BreathToRPM * MaxBreathe);
  }

void setOurHeart() {
  // not much to do, runBeat() does the actual beating
  CurrentBeatRate = map(BPM, MinBPM, MaxBPM, MinOurBeat, MaxOurBeat);
  }


class ExponentialSmooth {
  // a simple formula to sort-of do averaging: exponential smoothing

  float smoothed; // need float for converging, i.e. no rounding loss funniness
  const float factor; // forces operations to float space, actually a whole number

  // "factor" is kind of like the the number of samples that are averaged.
  // So, "5" is sort of like taking 5 samples and averaging them.
  ExponentialSmooth(const int factor) : factor(factor) {};

  operator int() const {return (int) smoothed;} // automatically let us be used as an int
  operator unsigned long() const {return (unsigned long) smoothed;} // automatically let us be used as unsigned long

  // we intend it to inline
  int average(const int raw_value) { 
    smoothed = raw_value / fast_factor + smoothed - smoothed / fast_factor; 
    return (int) smoothed;
    }

  }

void processPulse() {
  // We'll compute BPM and derivatives: BPM_Direction
  
  // debounce needs to be a number much larger than the interval between calls to us
  const int beat_debounce_bpm = 500; // take the fastest heartrate (~120) times about 4 to give a debounce
  const int beat_debounce = (1 / beat_debounce_bpm) * 1000; // convert debounce-bpm to millis
  static unsigned long last_beat = millis(); // arbitrary initial value
  static unsigned long beat_debounce_expire = 0; // we'll take first beat

  int pulse = analogRead(PulseSensorPin);
  bool during_beat = pulse > threshold;
  int beat_interval; // here for debug printing, magnitude should be small'ish delta-times

  if (during_beat) {
    beat_debounce_expire = now + beat_debounce; // keep track of "debounce"

    unsigned long now = millis(); // get it once

    // we invert debounce, must be "off" for the debounce period before a new beat counts
    if (now > beat_debounce_expire ) {
      // a new beat
      digitalWrite(LED_DuringBeat, HIGH); // really only need to turn it on now

      beat_interval = now - last_beat;

      // BPM and other smoothers
      for (ExponentialSmooth &smoother : BPM_Smoothers) {
        smoother.average( beat_interval );
        }

      // derivatives
      // by comparing a "fast" smooth with a slower smooth, we get a sort-of 1st derivative
      BPM_Direction = BPM - BPM_SmoothLong( beat_interval ); 
      }
    // else, wasn't a "new" beat, just "during" current beat
    }
  else {
    // not during beat, "debounce" will eventually expire
    digitalWrite(LED_DuringBeat, LOW); // wasteful: everytime
    }

  // Beat info, suitable for graphing: pulse,beat,bpm
  Serial.print(pulse);
  Serial.print(" ");
  Serial.print( during_beat * Threshold ); // during-beat is on/off, scale it to the threshold so it's reference/visible.
  Serial.print(" ");
  Serial.println(beat_interval); // raw time between beats, around 500 millis (but prints as spike)
  Serial.print(" ");
  Serial.println((BPM.smoothed); // BPM will have a different scale than pulse: ~ 80..120
  Serial.print(" ");
  Serial.println(BPM_Direction); // BPM will have a different scale than pulse: ~ 80..120

}
  

