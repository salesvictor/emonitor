/* 
 *  EMBEDDED SOFTWARE FOR THE E-MONITOR
 *  Created by Guilherme Leão, 21-11-2020
 *  Written by Guilherme Leão and Felipe Bizarro
 *  as part of group 1's HID-65 project:
 *  
 *  Group:
 *    Diego Teixeira Barreto Lima
 *    Eduardo Alarcon Mady Barbosa
 *    Felipe Bizarro Nini
 *    Gabriela Loiola Vilar
 *    Guilherme Pinheiro Cordeiro Leão
 *    Luiz Felipe Schiaveto
 *    Victor da Rocha Sales
 *  
 *  OBS.: Calc of the burden resistor:
 *
 *  Max and min values read by the sensor are:
 *  2.5 V +- I_sensor_max/kBurdenRes, I_sensor_max = 50 * sqrt(2) mA
 *
 *  If we want the max and min values equals to 5V and 0V => kBurdenRes approx. 33 Ohms
*/

#include <LiquidCrystal.h>
#include <SPI.h>
#include <Wire.h>           // For I2C protocol
#include <ArduinoJson.h>    // For converting the data Structure

#include "DataSet.h"
#include "EmonLib.h"        // Current sensor lib
#include "PushButton.h"
#include "RTClib.h"         // For clock

/* DEFINING PINS */
#define SCT_PIN A1          // Current sensor
#define LCD_SWITCH_PIN 2

/* CONSTANTS */
constexpr int   kDataPerFile              = 30;
constexpr unsigned long kDataReadInterval = 1000;    // In millisecond
constexpr unsigned long kLcdOnInterval    = 10000;   // In millisecond
constexpr float kBurdenRes                = 33.0f;   // In Ohms
constexpr float kInputIRmsMax             = 100.0f;  // In Amperes
constexpr float kInputVRms                = 127.0f;  // In Vrms
constexpr float kSctFactor                = 0.0005f;
constexpr float kSctTurns                 = 2000.0f; // Number os turns in the SCT-013 secondary

/* OBJECTS DECLARATIONS */
//DataSet measuresToSend(MESUR_PER_FILE);
EnergyMonitor emon1;                      // Object to handle the current sensor
RTC_DS1307 rtc;                           // Object to handle the DS1307 clock
LiquidCrystal lcd(9, 8, 6, 5, 4, 3);      // Pins connected to the LCD
PushButton switchButton(LCD_SWITCH_PIN);  // Object to handle the LCD switch button

DataSet<kDataPerFile> toSend; // Data to be converted to JSON
StaticJsonDocument<600> doc;  // Data converted to JSON

/* DATA READ */
float mIrms;          // In Arms
float mPower;         // In kW
DateTime currentTime; // RCT times are saved here

/* AUXILIAR VARIABLES */
int dataReadCount = 0;
unsigned long dataPreviousMillis = 0;
unsigned long lcdPreviousMillis  = 0;
unsigned long currentMillis;
bool lcdIsOn = false;

void setup()
{
  /* Initializind I2C communication */
  Wire.begin();

  /* Initializing serial port */
  // NOTE: Please doc what 9600 is
  Serial.begin(9600);
  Serial.print("INITIALIZING E-MONITOR...");

  /* Defining INPUT and OUTPUT pins */
  pinMode(SCT_PIN, INPUT);

  /* Initializing LCD */
  lcd.begin(16, 2);

  /* Initiliazing current handler */
  double cal = kSctTurns / kBurdenRes;  // Calibration parameter
  emon1.current(SCT_PIN, cal);

  /* Initializing push button */
  //PushButton
  //attachInterrupt(digitalPinToInterrupt(LCD_SWITCH_PIN), turnLCDOn, HIGH);

  /* Initilizing clock */
  if (!rtc.begin()) {
    Serial.println();
    Serial.println("ERROR: clock could not be initialized!");
    while (1); // Stop running
  }
  else
  {
    //rtc.adjust(DateTime(2020, 11, 21, 19, 24, 00)); // Adjust clock date and time
  }

  lcd.noDisplay();
  
  Serial.println("DONE");
}

void loop ()
{
  currentMillis = millis();

  // Checking if data should be sent:
  if (dataReadCount == kDataPerFile)
  {
    toSend.setFirstEpoch(rtc.now().unixtime());
    exportToJSON();
    dataReadCount = 0;
  }

  // Checking if data should be read:
  if (currentMillis - dataPreviousMillis >= kDataReadInterval)
  {
    dataPreviousMillis = currentMillis;

    // NOTE: Please doc what 1480 and 1000.0f are
    mIrms  = emon1.calcIrms(1480);
    mPower = mIrms * kInputVRms / 1000.0f; // Power in kW
    
    toSend.addData({mIrms, mPower});
    dataReadCount++;
    
    printDataToLCD(mIrms, mPower); // Printing current and power on the LCD
  }

  // Checking if LCD should be turned on:
  if (switchButton.isPressed())
  {
    turnLCDOn();
  }

  // Checking if LCD should be turned off:
  if (lcdIsOn && currentMillis - lcdPreviousMillis >= kLcdOnInterval)
  {
    turnLCDOff();
  }
}

void printDataToLCD(double current, double power)
{
  lcd.clear();
  lcd.print("Current: ");
  lcd.print(current);
  lcd.print(" A");

  lcd.setCursor(0, 1);
  lcd.print("Power: ");
  lcd.print(power);
  lcd.print(" kW");
}

void exportToJSON()
{
  // Allocate the JSON document
  //
  // Inside the brackets, 500 is the RAM allocated to this document.
  // Don't forget to change this value to match your requirement.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  
  auto& data = toSend.getData();
  doc.clear();
  doc["initTime"] = data.first_epoch;
  JsonArray currentData = doc.createNestedArray("current");
  JsonArray powerData = doc.createNestedArray("power");
  
  for (int i = 0; i < kDataPerFile; i++) { 
    currentData.add(data.current[i]);
    powerData.add(data.power[i]);
  }
  
  // Generate the minified JSON and send it to the Serial port.
  //serializeJson(doc, Serial); //Without \n

  // Generate the prettified JSON and send it to the Serial port.
  serializeJsonPretty(doc, Serial); //With \n
}

void turnLCDOn()
{
  lcdPreviousMillis = millis();
  lcdIsOn = true;
  lcd.display();
}

void turnLCDOff()
{
  lcdIsOn = false;
  lcd.noDisplay();
}
