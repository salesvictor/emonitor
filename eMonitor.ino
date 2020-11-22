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

#include "DataSet.h"
#include "EmonLib.h"        // Current sensor lib
#include "PushButton.h"
#include "RTClib.h"         // For clock

/* DEFINING PINS */
#define SCT_PIN A1         // Pino 15 - Current read
#define LCD_SWITCH_PIN 2

/* DEFINING CONSTANTS */

/* CONSTANTS */
constexpr int   kDataPerFile      = 60;
constexpr long  kDataReadInterval = 1000;    // In millisecond
constexpr long  kLcdOnInterval    = 10000;   // In millisecond
constexpr float kBurdenRes        = 33.0f;   // Ohms
constexpr float kInputIRmsMax     = 100.0f;  // Amperes
constexpr float kInputVRms        = 127.0f;  // Vrms
constexpr float kSctFactor        = 0.0005f;
//const float vcc = 5.0; // In Volts
//const float R1 = 10 ^ 4, R2 = 10 ^ 4; // In Ohms

/* OBJECTS DECLARATIONS */
//DataSet measuresToSend(MESUR_PER_FILE);
EnergyMonitor emon1;                      // Object to handle the current sensor
RTC_DS1307 rtc;                           // Object to handle the DS1307 clock
LiquidCrystal lcd(9, 8, 6, 5, 4, 3);      // Pins connected to the LCD
PushButton switchButton(LCD_SWITCH_PIN);

DataSet<kDataPerFile> toSend; // Data to be converted to JSON

/* DATA READ */
float mIrms;
float mPower;
DateTime currentTime;

//float tarifa = 0.3907;

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
  pinMode(SCT_PIN, INPUT);  // define o pino 15 como entrada

  /* Initializing LCD */
  lcd.begin(16, 2);

  /* Initiliazing current handler */
  // NOTE: Please doc what 2000 is
  double cal = 2000 / kBurdenRes;  // Calibration parameter
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

  //measuresToSend.initialize(60);
  Serial.println("DONE");

  lcd.noDisplay();
}

void loop ()
{
  currentMillis = millis();

  // Checking if data should be sent:
  if (dataReadCount == kDataPerFile)
  {
    exportToJSON();
    toSend.setFirstEpoch(now());
  }

  // Checking if data should be read:
  if (currentMillis - dataPreviousMillis >= kDataReadInterval)
  {
    dataPreviousMillis = currentMillis;

    // NOTE: Please doc what 1480 and 1000.0f are
    mIrms  = emon1.calcIrms(1480);
    mPower = mIrms * kInputVRms / 1000.0f;
    
    toSend.addData({mIrms, mPower};
    dataReadCount++;
    
    printDataToLCD(mCurrent, mPower); // Printing current and power on the LCD
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
  for (int i = 0; i < kDataPerFile; ++i)
  {
    Measure data = toSend.getData(i);
    Serial.print("    Current: ");
    Serial.print(data.current);
    Serial.println(" A");
    Serial.print("    Power: ");
    Serial.print(data.power);
    Serial.println(" kW");
  }
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
