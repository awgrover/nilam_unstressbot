/*
  Code for Nilam Sari.

  Designed for 2 communicating mega256's.
  #1 is the master, it reads the bpm, does all the calculations, and runs everything except eye #2
  #2 is a the slave, it gets the eye calculation and displays the correct eye #2 image.
  "Slave" is signaled by testing pin 12 for low (by setting it input-pullup): so ground 12 for slave.
  Same .ino so we don't have to worry about which code goes where.
  Same sd-card data, likewise.
  Communication is via serial on TX1, RX1.

  Everything uses the pulse sensor beats-per-minute, bpm.

  * LCD shows different images based on the bpm
  * MP3's are played when the bpm changes
  * Breathing is based on bpm
  * Robot-heart beats based on bpm

  Plan:
  # strip back to BPM
  # make a fake BPM
    set input_pullup, touch usb-ground:no-peak, otherwise random peaks > 1009..10010
    # looks like it works
  # play with pulsesensor: change smoothing, it's too slow ExponentialSmooth
  # add lcd
    * How to do 2 lcds?
      it's spi. board 1 could be the sd card, which needs 
      bus: MISO, MOSI and SCK
      CS is per board
      MISO is SD only
      DATA lines are bus
    * getting failure with any cables
      * impedence? driving-end terminating resistors? 100ohm? "source termination". in series
        e.g. impedence matching
      * spi mode? uh. yes? 
      * CS should go low for select
      * 
  * dual-mega communications
    * slave detect
    * master sends picture index
    * slave uses it
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

// uncommenting this turns on some code that fakes bpm during early development
// #define FAKEBPM
// uncomment this to output only raw pulse sensor values
// #define RAWPULSE

// Slave/master
boolean is_master;
const int SlaveDetect = 12;

// Pulse sensor
// https://www.adafruit.com/product/1093
//
const int PulseSensorPin = A0;
const int LED_DuringBeat = 13; // turn on this LED whenever the pulse-value > threshold. ie. _during_ beat
const int BeatDebounceBPM = 300; // take the fastest heartrate (~120) times about 4 to give a debounce
const int BeatDebounce = (1.0 / BeatDebounceBPM) * 60000L; // convert debounce-bpm to millis
#ifdef FAKEBPM
  const int Threshold = 1013;
  const int BeatAveraging = 5;
#else
  const int Threshold = 700; // 
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
const int BreatheMotorPin = A1; // pwm control for a cam motor
const int MinBreatheMotorPWM = 200; // slowest breathing "speed": ~= MinBPM
const int MaxBreatheMotorPWM = 1023; // fastest breathing "speed": ~= MaxBPM
int CurrentBreathingRate = MinBreatheMotorPWM; // got to start somewhere

// our heart just maps BPM to rate
const int HeartMotorPin = A2;
const int MinHeartMotorPWM = 200; // slowest heart "speed": ~= MinBPM
const int MaxHeartMotorPWM = 1023; // fastest heart "speed": ~= MaxBPM
int CurrentHeartRate = MinHeartMotorPWM; // got to start somewhere

// Eyes (lcd)
// http://www.14core.com/driving-the-qd320db16nt9481ra-3-2-tft-480x320-lcd-ili9481-wd-mega/
// NB: the links are labeled backwards on the site
// BUT, we are going to use the library at
// https://github.com/Bodmer/TFT_HX8357
// * "download" the zip
// * sketch:include library:add *.zip library...
// I don't think you need the adafruit gfx library
//
#include <TFT_HX8357.h>
//
// board says: "HVGA" "480x320" "3.2 TFTLCD Shield for arduino mega2560" "HX8357C"
// Seems to work with "model" ILI9481, and HX8357C
// The board does NOT support SPI to the LCD (the built-in SD card IS SPI only).
extern uint8_t SmallFont[]; // for printing stuff to lcd
// The lcd shield lines up with the mega256's pins.
// the Bodmer library uses the 16-bit databus, no spi for the LCD.
// So, one LCD per mega.
TFT_HX8357 eye = TFT_HX8357(); // atypical instantiation
// choice of flag seems to have no effect on drawing speed
const int BottomUp_BMP=1; // flag for drawing images "standard Bottom-Up bit maps"
const int TopDown_BMP=1; // flag for drawing images "inverted Top-Down bitmaps"

#include <SD.h>
// The built in SD card is SPI only
// lcd-CS=40, sd-card-CS=53
const int LCD_CS = 40; // But, I don't think SPI works for the LCD at all
const int SDCard_CS = 53;

// In order of the eye pictures ("eye_0.bmp", "eye_1.bmp"...):
// Give the corresponding maximum BPM. e.g. { 50, 100 } means: use "eye_1" up through 50 bpm, then "eye_2" up through 100 bpm
// Probably have 0..min, "absurdly low" and "absurdly high" values
// first BPM WILL happen when the pulse-sensor is first attached!
// FIXME: adaptive: start with whatever the person has, each step is +- 5%?
const int EyeBins[] = { 40, 60, 70, 80, 90, 100, 200 };
char eye_file_name[]="eye_0.bmp"; // will be modified as needed
const int eye_digit_index = 4; // which char to change
// Eye images are files named "eye_0.bmp" through "eye_7.bmp"
// They are 480x320, 24 bit/pixel, no-compression, .bmp
// see "setRotation" below if they are upside down.
// The imagemagick command to make them is
// mogrify -format bmp -type truecolor -depth 24 -compress None eye*.png
// Files size will be exactly: 460938 bytes ( 480 * 320 * 3 + 138 )

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
  Serial.println("Setup");

  // slave detect
  pinMode( SlaveDetect, INPUT_PULLUP );
  is_master = digitalRead(SlaveDetect); // high==open==master
  Serial.println( is_master ? F("MASTER") : F("SLAVE") );

  if (is_master) {
    pinMode( LED_DuringBeat, OUTPUT );

    pinMode( BreatheMotorPin, OUTPUT);
    pinMode( HeartMotorPin, OUTPUT);
    pinMode( PulseSensorPin, INPUT);
    #ifdef FAKEBPM
      pinMode( PulseSensorPin, INPUT_PULLUP ); // treat the pin like a bad cap sensor, i.e. antenna
    #endif
  }


  // Everybody does this

  // We will always use the sdcard
  if (SD.begin(SDCard_CS)) {
    Serial.println(F("SDCard 'begin'"));
  }
  else {
    Serial.println(F("SD.begin failed"));
  }

  eye.init();
  eye.fillScreen(TFT_BLACK);
  eye.drawLine(0,0, 320, 480, TFT_RED);

  // 3: original image is landscape, will be upright when USB is to the left
  // 1: original image is landscape, will be upright when USB is to the right
  eye.setRotation(3);

  //eye1.setFont(SmallFont); // debug/dev uses fonts
  //eye1.clrScr();
  //eye1.print((char*)"Nilam Unstressbot", CENTER, 1);

  updateEyes(0);

  Serial.println("Start");
}

void loop() {
  bool changed = processPulse(); // let's process it as often as we can, each time through the loop
  // FIXME only if changed most of these
  
  // Some things need maintenance: "keep it running"
  runBreathing();
  runHeart();
  // OurHeart.run();
  // playSound();

  if (changed) {
    int picture = setEyes();
    setOurHeart(); // sets CurrentHeartRate
    setBreathingRate(); // sets CurrentBreathingRate

    // beat
    #ifndef RAWPULSE
      if (millis() > 2000) {
        // skip printing noise at startup
        Serial.print(BPM.smoothed()); Serial.print(" ");
        Serial.print(BPM_SmoothLong.smoothed()); Serial.print(" ");
        Serial.print(sgn(BPM_Direction) * 10 + 100); Serial.print(" "); // BPM will have a different scale than direction

        // eyes
        Serial.print(BPM_ForEyes.smoothed()); Serial.print(" ");
        Serial.print(picture*2 + 10); Serial.print(" ");

        // our breathe
        Serial.print(map(CurrentBreathingRate,MinBreatheMotorPWM, MaxBreatheMotorPWM,0,20)); Serial.print(" ");

        // our heart
        Serial.print(map(CurrentHeartRate,MinHeartMotorPWM, MaxHeartMotorPWM,20,50)); Serial.print(" ");

        Serial.println();
        }
    #endif
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
    // the pulsesensor tends to go high when removed, for about 3-4 seconds
    // care?

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
  #ifdef RAWPULSE
    Serial.print(pulse);
      Serial.print(" ");
    Serial.print( during_beat * Threshold ); // during-beat is on/off, scale it to the threshold so it's reference/visible.
      Serial.print(" ");
    Serial.print(beat_interval); // raw time between beats, around 500 millis (but prints as spike)
      Serial.print(" ");
    Serial.println();
    delay(10); // scrolls too fast otherwise
  #endif

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
  // FIXME: Do it in an interrupt routine...
  Serial.print("Picture");
  Serial.println(picture_index);

  // we assume 1 digit
  eye_file_name[ eye_digit_index ] = '0' + picture_index;
  Serial.println(eye_file_name);
  drawBMP(eye_file_name, 0, 0, BottomUp_BMP);
  }

void setOurHeart() {
  // not much to do, runBeat() does the actual beating
  // FIXME: just use exact BPM? or, slightly slower BPM
  CurrentHeartRate = map(BPM.smoothed(), MinBPM, MaxBPM, MinHeartMotorPWM, MaxHeartMotorPWM);
  // maybe a "sleep" rate if bpm == 0
  }

void setBreathingRate() {
  // not much to do, runBreathing() does the actual breathing
  CurrentBreathingRate = map(BPM.smoothed(), MinBPM, MaxBPM, MinBreatheMotorPWM, MaxBreatheMotorPWM);
  // map doesn't constrain
  CurrentBreathingRate = constrain( CurrentBreathingRate, MinBreatheMotorPWM, MaxBreatheMotorPWM);
  // maybe a "sleep" rate if bpm == 0
  }

void runBreathing() {
  // breathing is a cam on a motor, so PWM driven
  // so, smoothly change speed from current to target
  // may need to update every loop
  static ExponentialSmooth motor_speed( 10 ); // the factor is how many samples to reach target: how smooth the change is

  // We expect to get called each loop, so use the smoothing to adjust speed towards target
  analogWrite(BreatheMotorPin, motor_speed.average( CurrentBreathingRate ) ); // pretty simple
  }

void runHeart() {
  // heart is a cam on a motor, so PWM driven
  // so, smoothly change speed from current to target
  // may need to update every loop
  static ExponentialSmooth motor_speed( 10 ); // the factor is how many samples to reach target: how smooth the change is

  // We expect to get called each loop, so use the smoothing to adjust speed towards target
  analogWrite(HeartMotorPin, motor_speed.average( CurrentHeartRate ) ); // pretty simple
  }

/*
void playSound() {
  // decide when to play some sound on the MP3 thingy
  static unsigned long next_sound = millis() + 2000; // don't bother at startup for 2seconds

  // FIXME: move to state machine
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

*/
