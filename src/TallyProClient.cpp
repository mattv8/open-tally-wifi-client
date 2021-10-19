///////////////////////////////////////////////////////////////////////
//     Arduino Script for Wi-Fi OBS Tally Light for Tally Arbiter    //
///////////////////////////////////////////////////////////////////////
//
//  Written by: Matthew P. Visnovsky
//  Adapted from https://github.com/josephdadams/TallyArbiter-M5StickCListener.
//  Includes code by Joseph Adams, Guido Visser, & Mark Kriegsman.
//
//  Requires the ESP32 board manager. In Arduino IDE, go to `File > Preferences`
//  Then enter https://dl.espressif.com/dl/package_esp32_index.json into the
//  “Additional Board Manager URLs” field. Wait for it to finish updating, then
//  go to `Tools > Board > Boards Manager`. Search for ESP32 and press install
//  button for the “ESP32 by Espressif Systems“. Finally, go to `Tools > Board
//  > ESP32 Arduino` and select TinyPICO as the current board.
//
    bool DEBUG = true; //Debug logging.
    bool writeDefaults = false; //Set to true to program default network setings on power-on
//
// -----------------------LIBRARIES------------------------- //
// Built-in libraries:
#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>

// Other libraries:
/* Arduino_JSON (ArduinoJson.h)
*  FastLED (FastLED.h)
*  MultiButton (PinButton.h)
*  SocketIOClient (SocketIoClient.h) https://github.com/timum-viw/socket.io-client
*  Websockets (Websockets.h) Required by SocketIOCLient, but not included here.
*/
#include <Arduino_JSON.h>
#include <FastLED.h>
#include <PinButton.h>
#include <SocketIoClient.h>
#include <WebSocketsClient.h>
#include <Preferences.h>

// -----------------------DEPENDENCY CHECK------------------------- //
#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif

// -----------------------USER CONFIG VARIABLES------------------------- //
/*  Change the following variables before compiling and sending the code to your device.
 */

//Default Wi-Fi SSID and password
char networkSSID[64] = "TallyServer";
char networkPass[64] = "12345";

//Default Tally Arbiter Server
char tallyarbiter_host[16] = "10.10.10.1";
int tallyarbiter_port = 4455;

//For static IP Configuration, change USE_STATIC to true and define your IP address settings below
bool USE_STATIC = false; // true = use static, false = use DHCP

IPAddress clientIp(192, 168, 2, 5); // Static IP
IPAddress subnet(255, 255, 255, 0); // Subnet Mask
IPAddress gateway(192, 168, 2, 1); // Gateway

// -----------------------PINS------------------------- //
#define LED_DATA      27 // PWM
#define RGB_LED_1     26 //
#define RGB_LED_2     34 //
#define RGB_LED_3     35 //
#define RGB_LED_4     37 //
#define EN_PIN         4 //
#define BUTTON_PIN     0 //

// TinyPico Battery (internal non-physical pins)
#define BAT_CHARGE 23
#define BAT_VOLTAGE 24

// -----------------------GLOBAL VARIABLES------------------------- //

//Time variables
unsigned long currentTime;  //Initialize master clock as global variable.
unsigned long pastTime = 0; // Set pastTime to 0. Don't redefine this anywhere or it will break the blink function.
unsigned long ticks;

// Battery variables
float batt = 0;
bool chg;

//Tally Arbiter variables
SocketIoClient socket;
JSONVar BusOptions;
JSONVar Devices;
JSONVar DeviceStates;
String deviceID = "unassigned";
String deviceName = "unassigned";
String listenerType = "Wi-Fi Tally Client";
bool mode_preview = false;
bool mode_program = false;

//General Variables
bool networkConnected = false;
bool socketConnected = false;
char buff[16]; //string to char buffer
int buttonState = 0;

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

//EEPROM variables
#define EEPROM_SIZE 1024 //Uses entirety of ESP32 flash mem
typedef struct  {
  char networkSSID[64];
  char networkPass[64];
  char tallyarbiter_host[16];
  int tallyarbiter_port;
  char deviceID[16];
  char deviceName[64];
} EEPROMdata;
EEPROMdata ROM; //Global object to store data read from EEPROM.

// State Variables
int SerialState = 0; int lastSerialState = 0;

// Serial Variables
char readString[128];
int startbit;
char Idx1[64];
char Idx2[64];
char Idx3[64];
char Idx4[64];

// ------------------------FUNCTIONS------------------------ //

/* Basic logger */
void logger(String strLog, bool DEBUG) {
  if (DEBUG == true) {
    Serial.println(strLog);
  }
}

/* Writes data to flash memory.
 *  See https://github.com/espressif/arduino-esp32/tree/master/libraries/EEPROM/examples
 */
void writeEEPROM() {
  int address = 0;
  EEPROM.writeString(address, networkSSID);  address += 64; // address = 64
  EEPROM.writeString(address, networkPass);  address += 64; // address = 128
  EEPROM.writeString(address, tallyarbiter_host);  address += 16; //address = 144
  EEPROM.writeInt(address, tallyarbiter_port);  address += 16; // address = 160
  EEPROM.writeString(address, deviceID);  address += 64; //address = 224
  EEPROM.writeString(address, deviceName); //address = 288
  EEPROM.commit();
  logger("Written data structure to EEPROM.",DEBUG);
}

/* Reads from flash memory
 *  See https://github.com/espressif/arduino-esp32/tree/master/libraries/EEPROM/examples
 */
void readEEPROM() {
  int address = 0; EEPROM.readString(address).toCharArray(ROM.networkSSID,64);
  address += 64;   EEPROM.readString(address).toCharArray(ROM.networkPass,64);
  address += 64;   EEPROM.readString(address).toCharArray(ROM.tallyarbiter_host,64);
  address += 16;   ROM.tallyarbiter_port = EEPROM.readInt(address);
  address += 16;   EEPROM.readString(address).toCharArray(ROM.deviceID,64);
  address += 64;   EEPROM.readString(address).toCharArray(ROM.deviceName,64);
  logger("Read network struct from EEPROM: ",DEBUG);
  logger("  SSID: "+String(ROM.networkSSID)+", Pass: "+String(ROM.networkPass),DEBUG);
  logger("  TAServer: "+String(ROM.tallyarbiter_host)+", TAPort: "+String(ROM.tallyarbiter_port),DEBUG);
  logger("  Device ID: "+String(ROM.deviceID)+", Device Name: "+String(ROM.deviceName),DEBUG);
}

/* Flash light a defined number of times and color. This takes `int spacing` ms processor time.
 *  Example usage: alertFlash(CRGB::White,3,5000,75);
 */
void alertFlash(CRGB Color, int num_flashes, int longInterval, int shortInterval, String message) {
  FastLED.setBrightness(DUTY_CYCLE*.50);//10% brightness
    //int shortInterval = 75; // Short interval between blinks

    if (currentTime - pastTime >= longInterval){// Long interval between flash events (fx input defined)
      pastTime = currentTime;// Reset long counters
      for(int count=1; count<=num_flashes; count++) { //flash light num_flahes times, quickly
        fill_solid( leds, NUM_LEDS, Color );       FastLED.show(); delay(shortInterval);
        fill_solid( leds, NUM_LEDS, CRGB::Black  );FastLED.show(); delay(shortInterval);
      }
    logger("A flash has occurred: "+message,DEBUG);
    }
}

////////////// Device Info functions ///////////////
String getBusTypeById(String busId) {
  for (int i = 0; i < BusOptions.length(); i++) {
    if (JSON.stringify(BusOptions[i]["id"]) == busId) {
      return JSON.stringify(BusOptions[i]["type"]);
    }
  }

  return "invalid";
}

void SetDeviceName() {

  int address;
  for (int i = 0; i < Devices.length(); i++) {
    if (JSON.stringify(Devices[i]["id"]) == "\"" + String(deviceID) + "\"") {
      String strDevice = JSON.stringify(Devices[i]["name"]);
      deviceName = strDevice.substring(1, strDevice.length() - 1);
      address = 224; EEPROM.writeString(address, deviceName); EEPROM.commit(); // Commit new deviceName to EEPROM
      break;
    }
  }

  logger("deviceID="+String(deviceID)+" deviceName="+String(deviceName),DEBUG);
}

////////////// Tally functions ///////////////
/* Evaluates whether the device is in program or preview
 * based on parsed JSON */
void evaluateMode() {
  FastLED.setBrightness(DUTY_CYCLE*.25);//25% brightness

  if (mode_preview && !mode_program) {//Device is in preview.
    logger("The device is in preview.", "info-quiet");
    fill_solid( leds, NUM_LEDS, CRGB::Green );
  }
  else if (!mode_preview && mode_program) {//Device is in program.
    logger("The device is in program.", "info-quiet");
    fill_solid( leds, NUM_LEDS, CRGB::Red );
  }
  else if (mode_preview && mode_program) {//Device is in preview & program.
    logger("The device is in preview+program.", "info-quiet");
    fill_solid( leds, NUM_LEDS, CRGB::Red );
  }
  else {// Device isn't in preview or program.
    fill_solid( leds, NUM_LEDS, CRGB::Black );
  }

  FastLED.show(); //Send LED color states to the LEDs
}

void processTallyData() {
  for (int i = 0; i < DeviceStates.length(); i++) {
    if (getBusTypeById(JSON.stringify(DeviceStates[i]["busId"])) == "\"preview\"") {
      if (DeviceStates[i]["sources"].length() > 0) {
        mode_preview = true;
      }
      else {
        mode_preview = false;
      }
    }
    if (getBusTypeById(JSON.stringify(DeviceStates[i]["busId"])) == "\"program\"") {
      if (DeviceStates[i]["sources"].length() > 0) {
        mode_program = true;
      }
      else {
        mode_program = false;
      }
    }
  }

  evaluateMode(); //
}

////////////// Socket Event Functions ///////////////
/* On connection to TA server */
void socket_Connected(const char * payload, size_t length) {
  String deviceObj = "{\"deviceId\": \"" + deviceID + "\", \"listenerType\": \"" + listenerType + "\"}";
  char charDeviceObj[128];
  strcpy(charDeviceObj, deviceObj.c_str());
  socket.emit("bus_options");
  socket.emit("device_listen_m5", charDeviceObj); //Send client type to TA server
  socketConnected = true;
}

void socket_Disconnected(const char * payload, size_t length) {
  socketConnected = false;
}

void socket_BusOptions(const char * payload, size_t length) {
  BusOptions = JSON.parse(payload);
}

void socket_Devices(const char * payload, size_t length) {
  Devices = JSON.parse(payload);
  SetDeviceName();
}

void socket_deviceID(const char * payload, size_t length) {
  deviceID = String(payload);
  SetDeviceName();
}

void socket_DeviceStates(const char * payload, size_t length) {
  DeviceStates = JSON.parse(payload);
  processTallyData();
}

/* flash the light white 3 times for socket event */
void socket_Flash(const char * payload, size_t length) {
  FastLED.setBrightness(DUTY_CYCLE*.25);//25% brightness
  for(int count=1; count<=10; count++) { //flash 10 times
    fill_solid( leds, NUM_LEDS, CRGB::White );FastLED.show();delay(100);
    fill_solid( leds, NUM_LEDS, CRGB::Black );FastLED.show();delay(100);
  }
  evaluateMode(); //return to listening mode
}

void socket_Reassign(const char * payload, size_t length) {
  String oldDeviceID = String(payload).substring(0,8);
  String newDeviceID = String(payload).substring(11);
  String reassignObj = "{\"oldDeviceId\": \"" + oldDeviceID + "\", \"newDeviceId\": \"" + newDeviceID + "\"}";
  char charReassignObj[1024];
  strcpy(charReassignObj, reassignObj.c_str());
  socket.emit("listener_reassign_object", charReassignObj);
  deviceID = newDeviceID;

  // Write new deviceID to EEPROM
  int address = 160; EEPROM.writeString(address, deviceID);
  EEPROM.commit();
  logger("Wrote newDeviceID to EEPROM.",DEBUG);

  SetDeviceName();
}

////////////// Networking Functions ///////////////
void connectToServer() {
  readEEPROM();
  logger("Connecting to Tally Arbiter host: " + String(ROM.tallyarbiter_host), DEBUG);
  socket.on("connect", socket_Connected);
  socket.on("bus_options", socket_BusOptions);
  socket.on("deviceId", socket_deviceID);
  socket.on("devices", socket_Devices);
  socket.on("device_states", socket_DeviceStates);
  socket.on("flash", socket_Flash);
  socket.on("reassign", socket_Reassign);
  socket.on("disconnect", socket_Disconnected);
  socket.begin(ROM.tallyarbiter_host, ROM.tallyarbiter_port);
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      logger("Network connected!", DEBUG);
      logger(WiFi.localIP().toString(), DEBUG);
      connectToServer(); //if connection to wifi is established, actually start to connect the socket
      networkConnected = true;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      logger("Network connection lost!", DEBUG);
      networkConnected = false;
      //logger("WiFi: Trying to reconnect", DEBUG);
      //WiFi.reconnect(); //if the connection to WiFi is lost (for whatever reason, try to reconnect
      break;
  }
}

void connectToNetwork() {
  readEEPROM();
  logger("Connecting to SSID: " + String(ROM.networkSSID), DEBUG);

  WiFi.disconnect(true);
  WiFi.onEvent(WiFiEvent);

  WiFi.mode(WIFI_STA); //station
  WiFi.setSleep(false);

  if(USE_STATIC == true) {
    WiFi.config(clientIp, gateway, subnet);
  }

  WiFi.begin(ROM.networkSSID, ROM.networkPass);

  delay(1000); //Delay is needed to actually figure out, if connection is established
}

////////////// FastLED Functions ///////////////
/* This function draws rainbows with an ever-changing, widely-varying set of parameters.
 * Function adapted from FastLED library: https://github.com/FastLED/FastLED */
void rainbow() {
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;

  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);

  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;

  for( uint16_t i = 0 ; i < NUM_LEDS*NUM_SETS; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);

    CRGB newcolor = CHSV( hue8, sat8, bri8);

    uint16_t pixelnumber = i;
    pixelnumber = (NUM_LEDS*NUM_SETS-1) - pixelnumber;

    nblend( leds[pixelnumber], newcolor, 64);
  }
}

/* Determine the charge and state of the battery. */
void battStatus() {

}

void powerOff() {
  if (currentTime - pastTime > 5000){// 5 second elapse
    pastTime = currentTime;// Reset all counters
    logger("Going to deep sleep...", DEBUG);
    delay(1000);
    digitalWrite(EN_PIN, LOW); // Go to deep sleep
    }
}

/* This function reads the incoming serial stream and parses it into respective variables.
 * We expect a string like  51,2,3,4*  or  50,2,*. For now, this only allows for up to 4
 * sources, but we can manually modify the code to allow for more.
 * Adapted from: https://forum.arduino.cc/index.php?topic=344010.0
 *   Serial Key:
 *   50  = Preview state change
 *   51  = Live state change
 *   0-3 = Live/Preview source range
 *   4   = Source is out of range
 *   ,   = Delimiter
 *   *   = Stop bit
*/
void readSerial () {
  char * ptr;  int charsRead; //Initialize function variables
  charsRead = Serial.readBytesUntil(',*', readString, sizeof(readString) - 1);
  readString[charsRead] = '\0';

  logger("Captured string is: "+String(readString),DEBUG);
  logger("Length of string is: "+String(strlen(readString)),DEBUG);

  ptr = strtok(readString, ","); if (ptr != '\0') { startbit = atoi(ptr); }
  ptr = strtok('\0', ",");       if (ptr != '\0') { strcpy(Idx1, ptr); } else { Idx1[0] = '\0'; }
  ptr = strtok('\0', ",");       if (ptr != '\0') { strcpy(Idx2, ptr); } else { Idx2[0] = '\0'; }
  ptr = strtok('\0', ",");       if (ptr != '\0') { strcpy(Idx3, ptr); } else { Idx3[0] = '\0'; }
  ptr = strtok('\0', ",");       if (ptr != '\0') { strcpy(Idx4, ptr); } else { Idx4[0] = '\0'; }

  readString[0] = '\0'; //clears variable for new input

  ////////////// Enter Programming Mode //////////////
  if (startbit == 99) {
    strcpy(networkSSID,Idx1); strcpy(networkPass,Idx2);
    strcpy(tallyarbiter_host,Idx3); tallyarbiter_port = atoi(Idx4);
    Serial.println(String(startbit)+","+String(Idx1)+","+String(Idx2)+","+String(Idx3)+","+String(Idx4)); //Send confirmation
    writeEEPROM(); // Commit changes to memory
    alertFlash(CRGB::Blue,1,1000,2000,"Entered programming mode");
    connectToNetwork(); // Try connecting to network again
    //connectToServer();  // Then try connecting to TallyArbiter again
  }

  ////////////// Enter Device Info Mode //////////////
  if (startbit == 98) {
    readEEPROM();
    Serial.println(String(startbit)+","
    +String(ROM.networkSSID)+","
    +String(ROM.networkPass)+","
    +String(ROM.tallyarbiter_host)+","
    +String(ROM.tallyarbiter_port)+","
    +String(ROM.deviceID)+","
    +String(ROM.deviceName)); //Send confirmation
    alertFlash(CRGB::Blue,1,1000,2000,"Entered device info mode");
  }

}

// ------------------------SETUP LOOP------------------------------- //

void setup() {

  // The EN pin MUST be set HIGH for circuitry to function.
  pinMode(EN_PIN,OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  pinMode(BUTTON_PIN, INPUT);

  Serial.begin(115200, SERIAL_8N1);
  while (!Serial);

  setCpuFrequencyMhz(80);    //Save battery by turning down the CPU clock
  btStop();                 //Save battery by turning off BlueTooth

  logger("Tally Arbiter Listener Client booting.", DEBUG);

  // FastLED Initializations.
  FastLED.addLeds<LED_TYPE,LED_DATA,COLOR_ORDER>(leds, TOTAL_LEDS)
    .setCorrection(TypicalSMD5050)
    .setDither(DUTY_CYCLE < 255);
  FastLED.setBrightness(DUTY_CYCLE*.75);//75% duty cycle

  // EEPROM Initializations
  EEPROM.begin(EEPROM_SIZE); // EEPROM initialization
  if (!EEPROM.begin(EEPROM_SIZE)){ logger("failed to initialise EEPROM",DEBUG); delay(10000); }
  if (writeDefaults) { writeEEPROM(); }//Write defaults on poweron if true

  // Network connectivity
  connectToNetwork(); //Starts Wifi connection
  while (!networkConnected) { //keep in while loop until connected to network
    rainbow(); FastLED.show();
    if(Serial.available()) { readSerial(); }
  }

  connectToServer(); //Connect to TA server

  delay(2000); //Safety bootup delay

}


// ------------------------VOID LOOP------------------------------- //

void loop() {

  // Master clock
  currentTime = millis();

  // Battery status check
  battStatus();

  // Power button check
  buttonState = digitalRead(BUTTON_PIN);
  if ( buttonState == HIGH ) {
    powerOff(); // Run powerOff() function.
    logger("Button state: HIGH",DEBUG);
  } else {
    logger("Button state: LOW",DEBUG);
  }

  // Alert flashes
  /* Run if the websocket connection to the TA server has been lost for some reason */
  if( !socketConnected ){ // Run if socket connection is lost
    alertFlash(CRGB::Orange,3,5000,75,"!socketConnected");
  }
  /* Run if device is connected to TA server, but has not been assigned an ID */
  if( ROM.deviceID == "unassigned" ) {
    alertFlash(CRGB::Magenta,3,3000,75,"Unassigned deviceID");
  }

  ////////////// Serial Communication ///////////////
  // Clear out all the LEDs upon serial state change
  SerialState = Serial.available();
  if (SerialState != lastSerialState) {
    FastLED.clear();
  }
  // Serial Communication
  if(Serial.available()) { // Check if recieving data

    readSerial(); // Record serial bits from incoming serial stream.

  }else { //If not recieving any serial data

    // Websocket loop
    socket.loop();

  }

  lastSerialState = SerialState; //Update serial state

}
