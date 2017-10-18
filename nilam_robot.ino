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
  * add lcd
  * incrementally do lcd
  * add LED analog for solenoid
  * add LED analog for cam-motor
  * map cam-motor to RPM for led
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


// Pulse sensor
// https://www.adafruit.com/product/1093
//
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
const int CurrentBeatRate = MinOurBeat; // got to start somewhere
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


void setup() {
  Serial.begin(9600);
  pinMode( LED_DuringBeat, OUTPUT );
}

void loop() {
  processPulse(); // let's process it as often as we can, each time through the loop
/*
  setEyes();
  setBreathingRate();
  runBreathing();
  setOurHeart();
  OurHeart.run();
  playSound();
*/
  }

void processPulse() {
  // We'll compute BPM and derivatives: BPM_Direction
  
  // debounce needs to be a number much larger than the interval between calls to us
  const int beat_debounce_bpm = 500; // take the fastest heartrate (~120) times about 4 to give a debounce
  const int beat_debounce = (1 / beat_debounce_bpm) * 1000; // convert debounce-bpm to millis
  static unsigned long last_beat = millis(); // arbitrary initial value
  static unsigned long beat_debounce_expire = 0; // we'll take first beat

  int pulse = analogRead(PulseSensorPin);
  bool during_beat = pulse > Threshold;
  int beat_interval; // here for debug printing, magnitude should be small'ish delta-times

  if (during_beat) {
    unsigned long now = millis(); // get it once

    beat_debounce_expire = now + beat_debounce; // keep track of "debounce"

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
      BPM_Direction = (int)BPM - BPM_SmoothLong.average( beat_interval ); 
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
  Serial.println((int)BPM); // BPM will have a different scale than pulse: ~ 80..120
  Serial.print(" ");
  Serial.println(BPM_Direction); // BPM will have a different scale than pulse: ~ 80..120

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
    if (BPM_ForEyes > binmax) {
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
  // maybe a "sleep" rate if bpm == 0
  }

void setOurHeart() {
  // not much to do, runBeat() does the actual beating
  // FIXME: just use exact BPM? or, slightly slower BPM
  CurrentBeatRate = map(BPM, MinBPM, MaxBPM, MinOurBeat, MaxOurBeat);
  // maybe a "sleep" rate if bpm == 0
  }


*/
