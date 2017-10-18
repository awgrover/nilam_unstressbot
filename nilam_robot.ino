/*
  Code for Nilam Sari.

  Everything uses the pulse sensor beats-per-minute, bpm.

  * LCD shows different images based on the bpm
  * MP3's are played when the bpm changes
  * Breathing is based on bpm
  * Robot-heart beats based on bpm

  Plan:
  * strip back to BPM
  * make a fake BPM
    set input_pullup, touch usb-ground:no-peak, otherwise random peaks > 1009..10010
    # looks like it works
  * add lcd
  * incrementally do lcd
    # pic value seems to track
    * setup/do eye load. rate limit: only 1/sec
  * add LED analog for solenoid
    # ourbeat rate seems ok
  * add LED analog for cam-motor
    # breathing rate seems ok
    * add led, with fakebpm range...
  * add sound sketch
  -- w/nilam
  * add lcd pics
  * add solenoid: TIP120
  * add cam-motor: TIP120
  * get mp3 player
  ---
  * add sound

*/

#include "state_machine.h" // awg's state machine stuff: https://github.com/awgrover/misc_arduino_tools
#include "ExponentialSmooth.h"

// uncommenting this turns on some code that fakes bpm
#define FAKEBPM

// Pulse sensor
// https://www.adafruit.com/product/1093
//
const int PulseSensorPin = A0;
const int LED_DuringBeat = 13; // turn on this LED whenever the pulse-value > threshold. ie. _during_ beat
const int BeatDebounceBPM = 300; // take the fastest heartrate (~120) times about 4 to give a debounce
const int BeatDebounce = (1.0 / BeatDebounceBPM) * 60000L; // convert debounce-bpm to millis
#ifdef FAKEBPM
  const int Threshold = 1009;
  const int BeatAveraging = 5;
#else
  const int Threshold = 512; // 
  const int BeatAveraging = 5; // adjust for stability/smoothing. for base BPM
#endif
int BPM_Direction = 0; // positive when BPM is going up, negative when going down, magnitude is how fast the change is. 
ExponentialSmooth BPM(BeatAveraging); // can use as an int
// Need min/max for breathing mapping, and our heart beat
const int MinBPM = 40; // lowest reasonable
const int MaxBPM = 180; // highest reasonable
const int DisconnectTimeout = (1.0 / (MinBPM/2)) * 60000L; // when it looks like someone let go
// Smoothing/BPM derivatives
// Make other smoothers for things that need them, put in the list for updating
ExponentialSmooth BPM_SmoothLong(BeatAveraging * 3); // for sort-of first-derivate. adjust for "inertia"
const int EyePersistance = BeatAveraging * 4; // smooth over ~4, need responsive, but not change too often
ExponentialSmooth BPM_ForEyes( BeatAveraging * 4 ); // smooth over ~4, need responsive, but not change too often

// Everything in this list wi1ll be updated on each beat, with the instantaneous BPM
ExponentialSmooth *BPM_Smoothers[] = { &BPM, &BPM_ForEyes, &BPM_SmoothLong }; // refs, no copy

// Breathing just maps BPM to breathing
const int MotorPin = A1; // pwm control for a cam motor
const int MinMotorPWM = 200; // slowest breathing "speed": ~= MinBPM
const int MaxMotorPWM = 1023; // fastest breathing "speed": ~= MaxBPM
int CurrentBreathingRate = MinMotorPWM; // got to start somewhere. rpms

// our heart just maps BPM to rate
const int MinOurBeat = 40; // lowest beat rate we want to do: correspond to MinBPM
const int MaxOurBeat = 80; // highest beat rate we want to do: correspond to MaxBPM
int CurrentBeatRate = MinOurBeat; // got to start somewhere
struct BeatParts {
  unsigned long hardDuration = 1000; // time it takes to drive solenoid fully out: "beat". and any hold time
  const int hardBeat = 1023; // the PWM value for a hard-beat "how hard". const for use in sm_analogWrite
  unsigned long hardRest = 59000; // time between beats.
    // total should be 60,000 millis (60 seconds)
    // expandable to "ba-dump" eventually
  };
constexpr BeatParts BeatExemplar;

// Define heart beat using statemachine
BeatParts Beat; // have to convert "per-minute" to durations, so calc later. start with exemplar
// SIMPLESTATEAS(start_hard_beat, (sm_analogWrite<PulseSensorPin, BeatExemplar.hardBeat>), hold_hard_beat)
// FIXME: rewrite with a fn that fetches beat SIMPLESTATEAS(hold_hard_beat, (sm_delay< Beat.hardDuration >), hard_relax)
// SIMPLESTATEAS(hard_relax, relaxHeart, hold_hard_relax)
// FIXME: rewrite with a fn that fetches beat SIMPLESTATEAS(hold_hard_relax, (sm_delay< Beat.hardRest >), start_hard_beat)
  // maybe add soft beat?
// STATEMACHINE(OurHeart, start_hard_beat);

// Eyes (lcd)
// http://www.14core.com/driving-the-qd320db16nt9481ra-3-2-tft-480x320-lcd-ili9481-wd-mega/
//
// In order of the eye pictures ("eye_1", "eye_2"...):
// Give the corresponding maximum BPM. e.g. { 50, 100 } means: use "eye_1" up through 50 bpm, then "eye_2" up through 100 bpm
// Probably have 0..min, "absurdly low" and "absurdly high" values
// first BPM WILL happen when the pulse-sensor is first attached!
const int EyeBins[] = { 40, 50, 60, 80, 120, 140, 200 }; 

// MP3 sounds
// https://www.adafruit.com/product/1381
//
enum SoundStates { SoundStart, SoundPlaying, SoundFinished, SoundDeciding };
SoundStates SoundState = SoundDeciding; // keep track of what we are doing, and coordinate with the interrupt routine
// Like EyeBins: "sound_1", "sound_2" correspond to BPMs
// E.g. { 40, 50, 60 } means play "sound_1" when BPM is <= 50
// NB: The first value is the cutoff for "play nothing", i.e. if bpm <= 40, play nothing.
const int SoundBins[] = { 40, 50, 60, 80, 120, 140, 200 };
const int SoundDecideWait = 2000; // after playing a sound, wait this long before DECIDING to play next one

// utility stuff
template <typename T> int sgn(T val) {
  // the sign of the value
  return (T(0) < val) - (val < T(0));
  }

void setup() {
  Serial.begin(115200);
  pinMode( LED_DuringBeat, OUTPUT );
  #ifdef FAKEBPM
    pinMode( PulseSensorPin, INPUT_PULLUP ); // treat the pin like a cap sensor
  #endif
  Serial.print("Heart Debounce ");Serial.println( BeatDebounce );
}

void loop() {
  bool changed = processPulse(); // let's process it as often as we can, each time through the loop
  // FIXME only if changed most of these
  
  // Some things need maintenance: "keep it running"
  runBreathing();
  /*
  OurHeart.run();
  playSound();
  */

  if (changed) {
    int picture = setEyes();
    setOurHeart(); // sets CurrentBeatRate
    setBreathingRate(); // sets CurrentBreathingRate

    // beat
    Serial.print(BPM.smoothed()); Serial.print(" ");
    Serial.print(BPM_SmoothLong.smoothed()); Serial.print(" ");
    Serial.print(sgn(BPM_Direction) * 10); Serial.print(" "); // BPM will have a different scale than direction

    // eyes
    Serial.print(BPM_ForEyes.smoothed()); Serial.print(" ");
    Serial.print(picture * 10); Serial.print(" ");

    // our breathe
    Serial.print(CurrentBreathingRate); Serial.print(" ");

    // our heart
    Serial.print(CurrentBeatRate); Serial.print(" ");

    Serial.println();
    }
  }

bool processPulse() {
  // We'll compute BPM and derivatives: BPM_Direction
  // We return true when we change the BPM
  
  // debounce needs to be a number much larger than the interval between calls to us
  static unsigned long last_beat = millis(); // arbitrary initial value
  static unsigned long beat_debounce_expire = 0; // we'll take first beat

  bool changed = false; // our signal

  int pulse = analogRead(PulseSensorPin);
  bool during_beat = pulse > Threshold;
  unsigned long beat_interval; // here for debug printing, magnitude should be small'ish delta-times

  unsigned long now = millis(); // get it once

  if (during_beat) {

    // we invert debounce, must be "off" for the debounce period before a new beat counts
    if (now > beat_debounce_expire ) {
      // a new beat
      digitalWrite(LED_DuringBeat, HIGH); // really only need to turn it on now

      beat_interval = now - last_beat; // in millis

      #ifdef FAKEBPM
        beat_interval *= 2; // noise is way too fast
      #endif

      // BPM and other smoothers
      for (ExponentialSmooth *smoother : BPM_Smoothers) {
        // we have interval, want BPM: interval 30 seconds, then 2bpm
        smoother->average( 60000L / beat_interval );
        }
      // Serial.println(); // debug

      // derivatives
      // by comparing a "fast" smooth with a slower smooth, we get a sort-of 1st derivative
      BPM_Direction = BPM.smoothed() - BPM_SmoothLong.smoothed();
      if (abs(BPM_Direction) < 2) BPM_Direction = 0;

      last_beat = now;
      beat_debounce_expire = now + BeatDebounce; // keep track of "debounce"

      changed = true;
      }
    // else, wasn't a "new" beat, just "during" current beat
    }
  else {
    digitalWrite(LED_DuringBeat, LOW); // wasteful: everytime

    // on disconnect, 0
    if ( now - last_beat > DisconnectTimeout && BPM.smoothed() != 0 ) {
      for (ExponentialSmooth *smoother : BPM_Smoothers) {
        smoother->reset(0);
        }
      changed = true;
      }

    }

  // Beat info, suitable for graphing: pulse,beat,bpm
  /*
  Serial.print(pulse);
    Serial.print(" ");
  Serial.print( during_beat * Threshold ); // during-beat is on/off, scale it to the threshold so it's reference/visible.
    Serial.print(" ");
  Serial.print(beat_interval); // raw time between beats, around 500 millis (but prints as spike)
    Serial.print(" ");
  */

  return changed;

}

int setEyes() {
  // use the BPM_ForEyes, pick an eye picture and display
  static int last_picture = -1; // impossible "last picture" for startup
  
  // We may need to rate limit...

  int which_picture = -1;
  int i=0;
  for( int binmax : EyeBins ) {
    // Serial.print("E");Serial.print(binmax);Serial.print(" ");
    if (binmax > BPM_ForEyes.smoothed() ) {
      which_picture = i;
      break; // we found the which_picture last time!
      }
    i += 1;
    }
  //Serial.println();

  // if we "fall" out (without using break), which_picture is -1, i is past the last one
  if (which_picture == -1 ) which_picture = i-1;

  if (which_picture != last_picture) {
    last_picture = which_picture;
    updateEyes( which_picture );
    }

  return which_picture;
  }

void updateEyes( int picture_index ) {
  // FIXME: pick "eye_$i", copy to lcd
  // Do it in an interrupt routine...
  // Serial.print("Picture");
  // Serial.println(picture_index);
  }

void setOurHeart() {
  // not much to do, runBeat() does the actual beating
  // FIXME: just use exact BPM? or, slightly slower BPM
  CurrentBeatRate = map(BPM.smoothed(), MinBPM, MaxBPM, MinOurBeat, MaxOurBeat);
  // maybe a "sleep" rate if bpm == 0
  }

void setBreathingRate() {
  // not much to do, runBreathing() does the actual breathing
  CurrentBreathingRate = map(BPM.smoothed(), MinBPM, MaxBPM, MinMotorPWM, MaxMotorPWM);
  // map doesn't constrain
  CurrentBreathingRate = constrain( CurrentBreathingRate, MinMotorPWM, MaxMotorPWM);
  // maybe a "sleep" rate if bpm == 0
  }

void runBreathing() {
  // breathing is a cam on a motor, so PWM driven
  // so, smoothly change speed from current to target
  // may need to update every loop
  static ExponentialSmooth motor_speed( 10 ); // the factor is how many samples to reach target: how smooth the change is

  // We expect to get called each loop, so use the smoothing to adjust speed towards target
  analogWrite(MotorPin, motor_speed.average( CurrentBreathingRate ) ); // pretty simple
  }

/*
void playSound() {
  // decide when to play some sound on the MP3 thingy
  static unsigned long next_sound = millis() + 2000; // don't bother at startup for 2seconds

  // while playing, or when it's time to decide...
  if (now > next_sound) {
    switch (SoundState) {

      SoundStart:
        // let it play
        break;

      SoundFinished:
        next_sound = now + SoundDecideWait; // careful, cf. with our enclosing "if".
        SoundState = SoundDeciding; //
        break;

      SoundDeciding:
        pickAndPlay();
        // FIXME: start interrupt player
        break;
      }
    }
  
  }

void pickAndPlay() {
  // use the BPM, pick a sound and start it
  static int last_sound = -1; // impossible "last picture" for startup
  
  int which_sound = 0;
  for( int &binmax : EyeBins ) {
    if (BPM_ForEyes.smoothed() > binmax) {
      break; // we found the which_sound last time!
      }
    which_sound = &binmax - &EyeBins[0]; // it's at least this bin
    }
  // if we "fall" out (without using break), which_sound is the last one

  // FIXME: lots of logic could go here:
  // if it's the same sound, wait twice as long to play it again
  // if it's the same sound, pick a "short" version: "Listen to my breathing..." then "ah"
  // pick a random sound from the "which_sound" group: "sound_1_1", "sound_2_2" etc.
  // use the direction: "That's better"
  // keep some state so we sound like we know what's been happening...

    last_sound = which_sound;
    SoundState = SoundStart;
    
    // FIXME: what do we do to start a sound? set a var? let interrupt do the right thing?

  }

void relaxHeart() {
  // Do the relax part of our heart, which includes recalculating rate

  analogWrite<PulseSensorPin, 0>
  // let's only recalculate the durations during a relax
  // CurrentBeatRate in per-minute
  // hardbeat..relax = 1 cycle
  // so scale the exemplar
  Beat.hardDuration = 1.0/CurrentBeatRate * BeatExemplar.hardDuration:
  Beat.hardRest = 1.0/CurrentBeatRate * BeatExemplar.hardRest;
  }


*/
