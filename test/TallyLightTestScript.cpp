///////////////////////////////////////////////////////////////////////
//     Arduino Script for Wi-Fi OBS Tally Light for Tally Arbiter    //
///////////////////////////////////////////////////////////////////////
//
//  Written by: Matthew P. Visnovsky
//  Includes code by Mark Kriegsman.
//
//  Requires the ESP32 board manager. In Arduino IDE, go to `File > Preferences`
//  Then enter https://dl.espressif.com/dl/package_esp32_index.json into the
//  “Additional Board Manager URLs” field. Wait for it to finish updating, then
//  go to `Tools > Board > Boards Manager`. Search for ESP32 and press install
//  button for the “ESP32 by Espressif Systems“. Finally, go to `Tools > Board
//  > ESP32 Arduino` and select TinyPICO as the current board.
//
    bool DEBUG = true; //Debug logging.

//
// Other libraries:
/* FastLED (FastLED.h)
*  TinyPICO Helper Library (TinyPICO.h)
*/
#include <Arduino.h>
#include <FastLED.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

// -----------------------PINS------------------------- //
#define LED_DATA      27 // PWM
#define RGB_LED_1     26 //
#define RGB_LED_2     34 //
#define RGB_LED_3     35 //
#define RGB_LED_4     37 //
#define BAT_VOLTAGE   32 //
#define EN_PIN         4 //
#define BUTTON_PIN     0 //

#define DEFAULT_VREF  1100  // Default referance voltage in mv
#define BATT_CHANNEL ADC1_CHANNEL_4  // Battery voltage ADC input

// -----------------------GLOBAL VARIABLES------------------------- //

// General variables
int buttonState = 0;

// Battery variables
uint32_t batt = 0;
bool chg;
unsigned long battSamples;
float lastBatChrg = 5;
float battavg;

//Time variables
unsigned long currentTime;  //Initialize master clock as global variable.
unsigned long battCheckTimer = 0; // Battery voltage check timer
unsigned long battChargeTimer = 0; // Battery charge status timer
unsigned long powerOffTimer = 0; // Power button timer

//FastLED Variables
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define DUTY_CYCLE  255                 //PWM duty cycle initialization
#define NUM_LEDS    4                   //Number of LEDs per tally lamp
#define NUM_SETS    1                   //Total number of tally lamps
#define TOTAL_LEDS  NUM_LEDS*NUM_SETS   //Total number of LEDs

//FastLED Data Structure Setup
CRGB rawleds[TOTAL_LEDS];
CRGBSet leds(rawleds, TOTAL_LEDS);

// ------------------------FUNCTIONS------------------------ //

/* Basic logger */
void logger(String strLog, bool DEBUG) {
  if (DEBUG == true) {
    Serial.println(strLog);
  }
}

/* Basic power-off */
void powerOff() {
  buttonState = digitalRead(BUTTON_PIN);
  if ( buttonState == HIGH ) {
    powerOffTimer = currentTime;// Reset all counters
    fill_solid( leds, NUM_LEDS, CRGB::Magenta ); FastLED.show();
  }
  else {
    if (currentTime - powerOffTimer > 2000){// 2 second elapse
      fill_solid( leds, NUM_LEDS, CRGB::Magenta ); FastLED.show();
      logger("Going to deep sleep...", DEBUG);
      delay(1000);
      fill_solid( leds, NUM_LEDS, CRGB::Green ); FastLED.show();
      digitalWrite(EN_PIN, LOW); // Go to deep sleep
    }
    fill_solid( leds, NUM_LEDS, CRGB::Yellow ); FastLED.show();
  }
}

/* Determine the charge and state of the battery. */
float battVoltage() {

  //Local Variables
  int R1 = 160; int R2 = 442; uint32_t raw;
  esp_adc_cal_characteristics_t chars;

  // Voltage Calculations
  analogRead(BAT_VOLTAGE);// Set up ADC with BAT_VOLTAGE pin
  raw = adc1_get_raw(BATT_CHANNEL);// Read of raw ADC value
  esp_adc_cal_characterize(ADC_UNIT_1,ADC_ATTEN_11db ,ADC_WIDTH_BIT_12,DEFAULT_VREF,&chars);// Get ADC calibration values
  batt += esp_adc_cal_raw_to_voltage(raw, &chars) * ((R1+R2) / R1);// Accumulate the calibrated voltage calculation

  battSamples++; //iterate sample counter

  if (currentTime - battCheckTimer > 2000){// Run this every 2 seconds
    battavg = ((float)batt / battSamples) / 1000; // Determine average, convert to volts
    battCheckTimer = currentTime; battSamples = 0; batt = 0; chg = 0;// Reset all counters
    logger("Battery voltage is: " + String(battavg) + " V", DEBUG);
  }
  return ( battavg );
}

void battChargeStatus() {
  float voltage = battVoltage();
  if (currentTime - battChargeTimer > 5000 && currentTime > 10000){// Run this every 2.5 seconds after 5 seconds of uptime
    if ( voltage != lastBatChrg && voltage > lastBatChrg ) {//Battery is charging
      logger("Battery is charging. Voltage is: " + String(voltage) + " V", DEBUG);
      if (voltage >= 4.2) {//Battery is fully charged
        fill_solid( leds, NUM_LEDS, CRGB::Green ); FastLED.show();
      } else {
        fill_solid( leds, NUM_LEDS, CRGB::Red ); FastLED.show();
      }
    } else {
      fill_solid( leds, NUM_LEDS, CRGB::White ); FastLED.show();
      logger("Not charging. Voltage is: " + String(voltage) + " V", DEBUG);
    }
    lastBatChrg = voltage;// Update charge state check
    battChargeTimer = currentTime;// Reset all counters
  }
}

// ------------------------SETUP LOOP------------------------------- //

void setup() {

  // The EN pin MUST be set HIGH for circuitry to function.
  pinMode(EN_PIN,OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  // Stagger LED power on
  pinMode(RGB_LED_1,OUTPUT);  digitalWrite(RGB_LED_1, LOW);  delay(100);
  pinMode(RGB_LED_2,OUTPUT);  digitalWrite(RGB_LED_2, LOW);  delay(100);
  pinMode(RGB_LED_3,OUTPUT);  digitalWrite(RGB_LED_3, LOW);  delay(100);
  pinMode(RGB_LED_4,OUTPUT);  digitalWrite(RGB_LED_4, LOW);  delay(100);

  analogReadResolution(4095);
  pinMode(BAT_VOLTAGE,INPUT);

  Serial.begin(115200, SERIAL_8N1);
  while (!Serial);

  setCpuFrequencyMhz(80);    //Save battery by turning down the CPU clock
  btStop();                 //Save battery by turning off BlueTooth

  logger("PCB Test script booting, please wait.", DEBUG);

  // FastLED Initializations.
  FastLED.addLeds<LED_TYPE,LED_DATA,COLOR_ORDER>(leds, TOTAL_LEDS)
    .setCorrection(TypicalSMD5050)
    .setDither(DUTY_CYCLE < 255);
  FastLED.setBrightness(DUTY_CYCLE*.75);//100% duty cycle

  delay(2000); //Safety bootup delay

}

void loop() {

  // Master clock
  currentTime = millis();

  // Battery status check
  //battChargeStatus();
  battVoltage();

  // Power button check
  powerOff();

  delay(50);

}
