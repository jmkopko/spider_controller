      /*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/*
  This example shows how to send HID (keyboard/mouse/etc) data via BLE
  Note that not all devices support BLE keyboard! BLE Keyboard != Bluetooth Keyboard
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined(ARDUINO_ARCH_SAMD)
  #include <SoftwareSerial.h>
#endif

#ifdef __AVR__
  #include <avr/power.h>
#endif


#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
#include "keycode.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
/*=========================================================================*/


// Create the bluefruit object, hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


// Set up keyboard report variables:
typedef struct
{
  uint8_t modifier;   /**< Keyboard modifier keys  */
  uint8_t reserved;   /**< Reserved for OEM use, always set to 0. */
  uint8_t keycode[6]; /**< Key codes of the currently pressed keys. */
} hid_keyboard_report_t;

// Report that sends to Central every scanning period
hid_keyboard_report_t keyReport = { 0, 0, { 0 } };

// Report sent previously. This is used to prevent sending the same report over time.
// Notes: HID Central intepretes no new report as no changes, which is the same as
// sending very same report multiple times. This will help to reduce traffic especially
// when most of the time there is no keys pressed.
// - Init to different with keyReport
hid_keyboard_report_t previousReport = { 0, 0, { 1 } };

#define undoReport {KEY_MOD_LCTRL, 0, {HID_KEY_Z, 0, 0, 0, 0, 0}};
#define redoReport {KEY_MOD_LCTRL, 0, {HID_KEY_Y, 0, 0, 0, 0, 0}}; 

// Set up GPIO pins for specific functions:
//Number of modifier keys:
const int NUM_MODS = 3;
//Number of regular keyboard keys:
const int NUM_KEYS = 3; //Maxiumum of 3; the final 3 slots of HID key report are reserved for Joystick and Scroll Wheel
// Number of macro keys; keys that will send combinations of multiple key presses
const int NUM_MACROS = 2;

//Modifier pin setup
int modPins[NUM_MODS]     = { 21, 22, 23 };
int modCodes[NUM_MODS] = {KEY_MOD_LCTRL, KEY_MOD_LSHIFT, KEY_MOD_LALT};

//General input pin setup
int inputPins[NUM_KEYS]     = { 5, 18, 11 };
int inputKeycodes[NUM_KEYS] = { HID_KEY_A, HID_KEY_I, HID_KEY_DELETE};

//Macro pin setup
int macroPins[NUM_MACROS]     = { 12       , 13       };
//int macroKeycodes[2] = {undoReport, redoReport};


// Joystick pin setup
const int Joystick_SW_pin = 18; // digital pin connected to switch output - currently handled by input pins
const int Joystick_X_pin = 19; // analog pin connected to X output
const int Joystick_Y_pin = 20; // analog pin connected to Y output


//Battery indicator: battery read pin, color variable, indicator LED:
int VBATPIN = 9;
uint16_t j = 0;
// Set up neopixel light on pin 6:
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 6, NEO_GRB + NEO_KHZ800);


// Rotary encoder pins and helpers
// usually the rotary encoders three pins have the ground pin in the middle
enum PinAssignments {
  encoderPinA = 2,   // center pin, green wire on proto
  encoderPinB = 3,   // left pin, blue wire on proto
  encoderButton = 5    // switch (separate button)
// connect the +5v and gnd appropriately - no +V; gnd is right pin, yellow wire
};

volatile unsigned int encoderPos = 0;  // a counter for the dial
unsigned int lastReportedPos = 1;   // change management
static boolean rotating=false;      // debounce management

// interrupt service routine vars
boolean A_set = false;              
boolean B_set = false;
//end rotary encoder pins and helpers

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);

   //set up the led
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'


  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    ble.factoryReset();
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Enable HID Service if not enabled */
  int32_t hid_en = 0;

  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Spider" )) ) {
    error(F("Could not set device name?"));
  }
  
  ble.sendCommandWithIntReply( F("AT+BleHIDEn"), &hid_en);

  if ( !hid_en )
  {
    Serial.println(F("Enable HID Service (including Keyboard): "));
    ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ));

    /* Add or remove service requires a reset */
    Serial.println(F("Performing a SW reset (service changes require a reset): "));
    !ble.reset();
  }
  
  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts keyboard input"));
  Serial.println();

  // Set up all pins
  //Mod Pins
  for(int i=0; i< NUM_MODS; i++)
  {
    pinMode(modPins[i], INPUT_PULLUP);
  }
  
  // Input Pins
  for(int i=0; i< NUM_KEYS; i++)
  {
    pinMode(inputPins[i], INPUT_PULLUP);
  }

  // Macro Pins
  for(int i=0; i< NUM_MACROS; i++)
  {
    pinMode(macroPins[i], INPUT_PULLUP);
  }
  //End GPIO Pin setup
  
  // Encoder interrupt setup
  pinMode(encoderPinA, INPUT_PULLUP); // new method of enabling pullups
  pinMode(encoderPinB, INPUT_PULLUP); 
  pinMode(encoderButton, INPUT_PULLUP);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);

  // End encoder setup
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{

  /* scan GPIO, since each report can have up to 6 keys
   * we can just assign a slot in the report for each GPIO 
   */
  if ( ble.isConnected() )
  {

    batteryUpdate();

    //Modifier logic
    bool modifierReset = true;

    for(int i=0; i< NUM_MODS; i++)
    {
      // GPIO is active low     
      if ( digitalRead(modPins[i]) == LOW )
      {
        keyReport.modifier |= modCodes[i];
        modifierReset = false;
      }
      
    }

    if(modifierReset) {
      keyReport.modifier = 0;
    }
    //End Modifier logic
    
    
    //Rotary Encoder logic
    rotating = true;  // reset the debouncer

    if (lastReportedPos != encoderPos) {
    //Serial.print("Testing Key Command:");
    //Serial.println(encoderPos, DEC);

    // Send key commands
      if (lastReportedPos < encoderPos) {
        //Serial.print("[");
        // need to report "[" keycode: 0x2f HID_KEY_BRACKET_LEFT
        keyReport.keycode[5] = HID_KEY_BRACKET_LEFT;
      }
      else if (lastReportedPos > encoderPos) {
        //Serial.print("]");
        // need to report "]" keycode: 0x30 HID_KEY_BRACKET_RIGHT
        keyReport.keycode[5] = HID_KEY_BRACKET_RIGHT;
      }
    
    lastReportedPos = encoderPos;
    } else {
      keyReport.keycode[5] = 0;
    }
  //End Rotary Encoder logic


  //Loops for standard input buttons    
    for(int i=0; i< NUM_KEYS; i++)
    {
      // GPIO is active low     
      if ( digitalRead(inputPins[i]) == LOW )
      {
        keyReport.keycode[i] = inputKeycodes[i];
      }else
      {
        keyReport.keycode[i] = 0;
      }
    }

    // Macro Buttons
      if ( digitalRead(macroPins[0]) == LOW )
      {
          keyReport = undoReport;
      }
      else if (digitalRead(macroPins[1]) == LOW)
      {
          keyReport = redoReport;
      }
  //End standard input loops

    //Joystick logic 
    if(analogRead(Joystick_X_pin) > 950) {
      keyReport.keycode[3] = HID_KEY_ARROW_RIGHT;
    } else if(analogRead(Joystick_X_pin) < 100) {
      keyReport.keycode[3] = HID_KEY_ARROW_LEFT;
    } else {
      keyReport.keycode[3] = 0;
    }
  
    if(analogRead(Joystick_Y_pin) > 950) {
      keyReport.keycode[4] = HID_KEY_ARROW_UP;
    } else if(analogRead(Joystick_Y_pin) < 100) {
      keyReport.keycode[4] = HID_KEY_ARROW_DOWN;
    } else {
      keyReport.keycode[4] = 0;
    }
    
    sendKeyReport();

    } //end of ifBLEconnected condition
 
   // scanning period is 5 ms
  delay(5);

} //end of main loop

// Interrupt on A changing state
void doEncoderA(){
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change? 
  if( digitalRead(encoderPinA) != A_set ) {  // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set ) 
      encoderPos += 1;
      
    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB(){
  if ( rotating ) delay (1);
  if( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if( B_set && !A_set ) 
      encoderPos -= 1;

    rotating = false;
  }
}

void sendKeyReport() {
  // Only send if it is not the same as previous report
      if ( memcmp(&previousReport, &keyReport, 8) )
      {
        // Send keyboard report
        ble.atcommand("AT+BLEKEYBOARDCODE", (uint8_t*) &keyReport, 8);

        // copy to previousReport
        memcpy(&previousReport, &keyReport, 8);
      }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void batteryUpdate() {
 //Update LED color and battery check
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  //Serial.print("VBat: " ); Serial.println(measuredvbat); //for debugging
  
  if (measuredvbat < 3.6) {
    strip.setPixelColor(0, 255, 0, 0);
  } else {
  
        j++;
        if (j < 256) {
        strip.setPixelColor(0, Wheel(j & 255));
        } else {
          j = 0;
        }
  }
  strip.show();
}


