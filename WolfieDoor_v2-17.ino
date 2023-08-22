#include <Wire.h> // Library for I2C communication
#include <Arduino.h>
#include "Adafruit_VL53L0X.h" // laser sensors
#include <LiquidCrystal_I2C.h> // Library for LCD
#include <DS3231.h> // For real-time clock. https://github.com/NorthernWidget/DS3231 
#include <EEPROM.h> // For saving configuration settings
#include "Timemark.h" // for events

// Recent updates:
//  2022.12.05:  Implemented config screens 4 and 5: detection sampling rate and buzzer.
//               Implemented auto-reset when outside sensor malfunctions (reset once)
//  DONE but hasn't been tested (12/5/22):  if Unlock in morning is set, play alarm sound when door is open -- 
// ALSO DONE BUT UNTESTED- reduced buzzer time. Changed LEDs so each blinks when the sensor is triggered. BLUE LED indicates lock on/off.
                // changed so unlocked in morning is not set until 4AM (instead of midnight)
                // turned off buzzer when config buttons are pressed
                // turn on sensor indicator LEDs before opening door
                // change inside sensor sampling rate to 1/2 the outside rate
                // fixed unlock-LED status when RTC is disabled
                // change LED colors-- green is OPEN. Yellow is INSIDE SENSOR. Blue is OUTSIDE SENSOR
                // TODO: add reset when inside sensor is triggered for more than 120 seconds? 
                // add beep on reset

Adafruit_VL53L0X laserSensor = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4); // Change to (0x27,16,2) for 16x2 LCD.
DS3231 realTimeClock; //  I2C address 0x68 is the DS3231 and 0x56 is the AT24C32.  

// EEPROM saved variables
#define EEPROM_OUTSIDE_SETPOINT_ADDR 0 // eeprom address. occupy 2 bytes per uint16
#define EEPROM_INSIDE_SETPOINT_ADDR 2
#define EEPROM_DOOR_LOCKED_HOUR_ADDR 4
#define EEPROM_DOOR_OPEN_HOUR_ADDR 6
#define EEPROM_DETECTION_SENSITIVITY_ADDR 8
#define EEPROM_OPEN_IN_MORNING_ADDR 10
#define EEPROM_RTC_ENABLED_ADDR 12
#define EEPROM_DETECTION_SAMPLING_RATE_ADDR 14
#define EEPROM_USE_BUZZER_ADDR 16
#define EEPROM_REBOOT_COUNTER_ADDR 18

// These vars are loaded from eeprom at startup
uint16_t mEEPROM_OutsideSetpoint; // set with calibrate screen
uint16_t mEEPROM_InsideSetpoint; // set with calibrate screen
uint16_t mEEPROM_DoorLockedHourIndex; // index into cTimeConfigurationsPM
uint16_t mEEPROM_DoorOpenHourIndex; // index into cTimeConfigurationsAM
uint16_t mEEPROM_DetectionSensitivityIndex; // index into cDetectionSensitivity
uint16_t mEEPROM_DetectionSamplingRateIndex; // index into cDetectionSensitivity
uint16_t mEEPROM_RebootCounter;
bool mEEPROM_UseBuzzer;
bool mEEPROM_UnlockInMorning;
bool mEEPROM_RTC_Enabled; // unused? use switch instead

// Misc defines
#define DEBUG false
#define SERIAL_BAUD_RATE 115200 // 9600
#define MUX_DELAY_MS 20
#define LARGE_DELAY 500
#define ON true
#define OFF false
#define OPEN true
#define CLOSED false
#define LEFT false
#define RIGHT true
#define INSIDE 0
#define OUTSIDE 1
#define INVALID_READING 0x1FFF

// Define arduino connection pins
#define RTC_DISPLAY_CHAN 0 // SD0 / SC0 on mux board
#define LASER_CHAN_0 2 // SD2 / SC2 on mux board
#define LASER_CHAN_1 4 // SD4 / SC4 on mux board
#define STEPPER_EN_PIN 30 // arduino I/o
#define STEPPER_STEP_PIN 28 // arduino I/o
#define STEPPER_DIR_PIN 26  // arduino I/o
#define USER_SWITCH_PIN_0 44 // use as output i/o pulled up
#define USER_SWITCH_PIN_1 45 // connect as input i/o
#define USER_SWITCH USER_SWITCH_PIN_1
#define USER_BUTTON_0_PIN_0 34 // use as output i/o pulled up
#define USER_BUTTON_0_PIN_1 35 // connect as input i/o
#define USER_BUTTON_0 USER_BUTTON_0_PIN_1
#define USER_BUTTON_1_PIN_0 38 // use as output i/o pulled up
#define USER_BUTTON_1_PIN_1 39 // connect as input i/o
#define USER_BUTTON_1 USER_BUTTON_1_PIN_1
#define LED_GREEN_POSITIVE_PIN 52
#define LED_GREEN_NEGATIVE_PIN 53
#define LED_YELLOW_POSITIVE_PIN 50
#define LED_YELLOW_NEGATIVE_PIN 51
#define LED_BLUE_POSITIVE_PIN 48
#define LED_BLUE_NEGATIVE_PIN 49
#define BUZZER_PIN 43
#define GREEN LED_GREEN_POSITIVE_PIN
#define BLUE LED_BLUE_POSITIVE_PIN
#define YELLOW LED_YELLOW_POSITIVE_PIN

bool mSwitchPosition = LEFT;
bool mButtonPressedTop = false;
bool mButtonPressedBottom = false;

#define STEPPER_STEP_LENGTH_US 300 // 1000 is ok -- 500 works well but is a little slow. 350 works.
#define STEPPER_NUM_STEPS_TO_OPEN_DOOR 2400 // 2400 is good when there is some slack in string

#define TIME_TO_CLEAR_DETECTED 50 // motion detect wait time - 200 is good but long
#define TIME_TO_CLEAR_NOT_DETECTED 5000 // motion detect off wait time - was 2000
Timemark MotionDetected(TIME_TO_CLEAR_DETECTED);
Timemark MotionNotDetected(TIME_TO_CLEAR_NOT_DETECTED);

#define MOTION_DETECT_LED_BLINK_RATE 75
bool mStateOfGreenLED = OFF;
bool mStateOfYellowLED = OFF;
bool mStateOfBlueLED = OFF;
Timemark BlinkGreenLED(MOTION_DETECT_LED_BLINK_RATE);
Timemark BlinkYellowLED(MOTION_DETECT_LED_BLINK_RATE);
Timemark BlinkBlueLED(MOTION_DETECT_LED_BLINK_RATE);

#define BUTTON_LONG_PRESS_TIME_MS 350 // 350 // 700 // need this delay for LCD to update
#define BUTTON_SHORT_PRESS_TIME_MS 40
#define MOTION_DETECT_DOOR_HELD_OPEN_TIME 15000
#define BUTTON_PRESSED_DOOR_HELD_OPEN_TIME 25000
Timemark DoorOpenedFromMotionDetect(MOTION_DETECT_DOOR_HELD_OPEN_TIME);
Timemark DoorOpenedFromButtonPress(BUTTON_PRESSED_DOOR_HELD_OPEN_TIME);
Timemark BottomButtonPress(BUTTON_SHORT_PRESS_TIME_MS);
Timemark BottomButtonHold(BUTTON_LONG_PRESS_TIME_MS);
Timemark TopButtonHold(BUTTON_SHORT_PRESS_TIME_MS); // BUTTON_LONG_PRESS_TIME_MS // i like short press better.
// Buzzer stuff
#define BUZZER_BUTTON_ON_TIME_DOOR_MS 750 // reduced from 1500 --> 1000
#define BUZZER_BUTTON_ON_TIME_CONFIG_MS 75
#define BUZZER_ALARM_ON_TIME_MS 150
#define BUZZER_ALARM_OFF_TIME_MS 8000
Timemark BuzzerAlarm(BUZZER_ALARM_OFF_TIME_MS);
Timemark BuzzerButton(BUZZER_BUTTON_ON_TIME_DOOR_MS);

// LCD stuff
#define TIME_TO_UPDATE_LCD 5000 // ms
#define TIME_TO_UPDATE_LCD_SENSORS 500 // ms
Timemark LcdUpdate(TIME_TO_UPDATE_LCD);
Timemark LcdUpdateSensorValuesOnly(TIME_TO_UPDATE_LCD_SENSORS);

const float cDetectionSensitivity[10] = {  0.10f, 0.15f, 0.20f, 0.25f, 0.30f, 0.35f, 0.40f, 0.45f, 0.5f  };
const uint8_t cDetectionSensitivity_size = sizeof(cDetectionSensitivity)/sizeof(float);
const uint16_t cDetectionSamplingRates[7] = {  40, 60, 80, 100, 120, 140, 160  };
const uint8_t cDetectionSamplingRates_size = sizeof(cDetectionSamplingRates)/sizeof(uint16_t);
#define SENSITIVITY_DEFAULT_INDEX 4 // default sensitivity index into cDetectionSensitivity upon initial programming
#define SAMPLERATE_DEFAULT_INDEX 1

#define RTC_BUTTON_UNLOCK_EARLIEST_AM_HOUR 4 // 4AM is the earliest time when pushing the button will switch RTC to UNLOCKED
const uint8_t cTimeConfigurationsAM[9][2] = { { 7, 0 }, { 7, 30 }, { 8, 0 }, { 8, 30 }, { 9, 0 }, { 9, 30 }, { 10, 0 }, { 10, 30 }, { 11, 0 } };
const uint8_t cTimeConfigurationsPM[13][2] = { { 5, 0 }, { 5, 30 }, { 6, 0 }, { 6, 30 }, { 7, 0 }, { 7, 30 }, { 8, 0 }, { 8, 30 }, { 9, 0 }, { 9, 30 }, { 10, 0 }, { 10, 30 }, { 11, 0 } };
const uint8_t cTimeConfigurationsAM_size = sizeof(cTimeConfigurationsAM)/sizeof(uint8_t)/2;
const uint8_t cTimeConfigurationsPM_size = sizeof(cTimeConfigurationsPM)/sizeof(uint8_t)/2;

// Real-time clock settings
bool bIsDoorAllowedOpen = false;
bool bUnlockedInMorning = false;
bool mClockCentury = false;
bool mClockFlag12H = true;
bool mClockFlagPM;
byte mCurrentHour;
byte mCurrentMinute;
char mCurDate[3]; // these need to be greater than 2? not sure why. newline char? otherwise there is weird behavior
char mCurMonth[3];
char mCurMinute[3];
char mCurHour[3];

unsigned long mTimeNow = 0; // beware calculations with int
unsigned long mTimeOfInsideMotionDetected = 0;
unsigned long mTimeOfOutsideMotionDetected = 0;
 // 4294967295; //unsigned long maximum value
#define MODE_MAIN 0
#define MODE_CALIBRATE 1
#define MODE_CONFIG_0 2
#define MODE_CONFIG_1 3
#define MODE_CONFIG_2 4
#define MODE_CONFIG_3 5
#define MODE_CONFIG_4 6
#define MODE_CONFIG_5 7
#define NUM_MODES 7
int mCurrentMode = MODE_MAIN;
unsigned long mModeChangeTime = 0;

// State machine stuff
#define STATE_DOOR_CLOSED 0
#define STATE_DOOR_OPENING 1
#define STATE_DOOR_OPEN 2
#define STATE_DOOR_CLOSING 3
int mCurrentDoorState = STATE_DOOR_CLOSED;  // assume bootup with door in closed position
int mNextDoorState = STATE_DOOR_CLOSED;

// Laser sensor defines
uint16_t mLaserDistanceInside = INVALID_READING;
uint16_t mLaserDistanceOutside = INVALID_READING;
String mInsideSensorVal = "    ";
char mInsideSensorValue[5]; // size must be 1 greater than the string length (e.g. 4 digits)
char mOutsideSensorValue[5];
char mInsideSensorSetpoint[5];
char mOutsideSensorSetpoint[5];
char mInsideSensorThreshold[5];
char mOutsideSensorThreshold[5];
bool mInsideSensorTriggered = false;
bool mOutsideSensorTriggered = false;
bool mMotionDetected = false;
float mInsideThreshold = 0.0f; // calculated after reading eeprom
float mOutsideThreshold = 0.0f;

bool mOutsideSensorBootOk = true;
bool mInsideSensorBootOk = true;

void(* resetFunc) (void) = 0;  // declare reset fuction at address 0

void setup() {
  // Drive one of each switch pins high to read as an input
  pinMode(USER_SWITCH_PIN_0, OUTPUT);
  pinMode(USER_BUTTON_0_PIN_0, OUTPUT);
  pinMode(USER_BUTTON_1_PIN_0, OUTPUT);
  digitalWrite(USER_SWITCH_PIN_0, LOW);
  digitalWrite(USER_BUTTON_0_PIN_0, LOW);
  digitalWrite(USER_BUTTON_1_PIN_0, LOW);
  // Set the other switch pins as inputs
  pinMode(USER_SWITCH_PIN_1, INPUT_PULLUP);
  pinMode(USER_BUTTON_0, INPUT_PULLUP);
  pinMode(USER_BUTTON_1, INPUT_PULLUP);

  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);

  // Set up LEDs
  pinMode(LED_GREEN_POSITIVE_PIN, OUTPUT);
  pinMode(LED_GREEN_NEGATIVE_PIN, OUTPUT);
  pinMode(LED_YELLOW_POSITIVE_PIN, OUTPUT);
  pinMode(LED_YELLOW_NEGATIVE_PIN, OUTPUT);
  pinMode(LED_BLUE_POSITIVE_PIN, OUTPUT);
  pinMode(LED_BLUE_NEGATIVE_PIN, OUTPUT);
  digitalWrite(LED_GREEN_NEGATIVE_PIN, LOW);
  digitalWrite(LED_YELLOW_NEGATIVE_PIN, LOW);
  digitalWrite(LED_BLUE_NEGATIVE_PIN, LOW);
  turnOffLED(GREEN);
  turnOffLED(BLUE);
  turnOffLED(YELLOW);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  // Initialize I2C and Serial
  Wire.begin();
  Serial.begin(SERIAL_BAUD_RATE);

  // Enable TCM2209 stepper motor driver by pulling ENABLE pin LOW
  digitalWrite(STEPPER_EN_PIN, LOW);

  Serial.println("Hello Luke.");
  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  Serial.println("Initializing TCA9548A I2C MUX");

  // Initiate the LCD:
  TCA9548A(RTC_DISPLAY_CHAN);
  lcd.init();
  lcd.backlight();
  //lcd.setCursor(0,0);
  //lcd.print("*** Wolfie Door ***");
  lcd.setCursor(0,0);  
  lcd.print("***** STARTUP! *****");
  lcd.setCursor(0,1); 
  lcd.print("Outside (mm): ");
  lcd.setCursor(1,2);
  lcd.print("Inside (mm): ");
  lcd.setCursor(0,3);
  lcd.print("******* Time *******");

  // Initialize inside sensor
  TCA9548A(LASER_CHAN_0);
  Serial.println(" VL53L0X test channel 0. (inside)");
  if (!laserSensor.begin()) {
    Serial.println(F("Failed to boot VL53L0X CHAN 0 (inside)"));
    mInsideSensorBootOk = false;
  //  while(1);
  }

  // Initialize outside sensor
  TCA9548A(LASER_CHAN_1);
  Serial.println(" VL53L0X test channel 1. (outside)");
  if (!laserSensor.begin()) {
    Serial.println(F("Failed to boot VL53L0X CHAN 1 (outside)"));
    mOutsideSensorBootOk = false;
  //  while(1);
  }
  
  // Read saved configuration from EEPROM, load into variables
  Serial.println(F("Loading EEPROM"));
  EEPROM.get(EEPROM_OUTSIDE_SETPOINT_ADDR, mEEPROM_OutsideSetpoint);
  EEPROM.get(EEPROM_INSIDE_SETPOINT_ADDR, mEEPROM_InsideSetpoint);
  EEPROM.get(EEPROM_DOOR_LOCKED_HOUR_ADDR, mEEPROM_DoorLockedHourIndex);
  EEPROM.get(EEPROM_DOOR_OPEN_HOUR_ADDR, mEEPROM_DoorOpenHourIndex);
  EEPROM.get(EEPROM_DETECTION_SENSITIVITY_ADDR, mEEPROM_DetectionSensitivityIndex);
  EEPROM.get(EEPROM_OPEN_IN_MORNING_ADDR, mEEPROM_UnlockInMorning);
  EEPROM.get(EEPROM_RTC_ENABLED_ADDR, mEEPROM_RTC_Enabled);
  EEPROM.get(EEPROM_DETECTION_SAMPLING_RATE_ADDR, mEEPROM_DetectionSamplingRateIndex);
  EEPROM.get(EEPROM_USE_BUZZER_ADDR, mEEPROM_UseBuzzer);
  EEPROM.get(EEPROM_REBOOT_COUNTER_ADDR, mEEPROM_RebootCounter);

  sprintf(mInsideSensorSetpoint, "%04d", (unsigned int)mEEPROM_InsideSetpoint); // get the string for display
  sprintf(mOutsideSensorSetpoint, "%04d", (unsigned int)mEEPROM_OutsideSetpoint);

  Serial.print("mEEPROM_OutsideSetpoint=");Serial.println(mEEPROM_OutsideSetpoint);
  Serial.print("mEEPROM_InsideSetpoint=");Serial.println(mEEPROM_InsideSetpoint);
  Serial.print("mEEPROM_DoorLockedHourIndex=");Serial.println(mEEPROM_DoorLockedHourIndex);
  Serial.print("mEEPROM_DoorOpenHourIndex=");Serial.println(mEEPROM_DoorOpenHourIndex);
  Serial.print("mEEPROM_DetectionSensitivityIndex=");Serial.print(mEEPROM_DetectionSensitivityIndex);Serial.print(" --> ");Serial.println(cDetectionSensitivity[mEEPROM_DetectionSensitivityIndex]);
  Serial.print("mEEPROM_UnlockInMorning=");Serial.println(mEEPROM_UnlockInMorning);
  Serial.print("mEEPROM_RTC_Enabled=");Serial.println(mEEPROM_RTC_Enabled);
  Serial.print("mEEPROM_DetectionSamplingRateIndex=");Serial.println(mEEPROM_DetectionSamplingRateIndex);
  Serial.print("mEEPROM_UseBuzzer=");Serial.println(mEEPROM_UseBuzzer);
  Serial.print("mEEPROM_RebootCounter=");Serial.println(mEEPROM_RebootCounter);
  Serial.println(F("Done Loading EEPROM"));



  if (mEEPROM_DetectionSensitivityIndex >= cDetectionSensitivity_size) mEEPROM_DetectionSensitivityIndex=SENSITIVITY_DEFAULT_INDEX;
  if (mEEPROM_DetectionSamplingRateIndex >= cDetectionSamplingRates_size) mEEPROM_DetectionSamplingRateIndex=SAMPLERATE_DEFAULT_INDEX;
  MotionDetected.limitMillis(cDetectionSamplingRates[mEEPROM_DetectionSamplingRateIndex]);

  mInsideThreshold = (float)mEEPROM_InsideSetpoint - ((float)mEEPROM_InsideSetpoint * cDetectionSensitivity[mEEPROM_DetectionSensitivityIndex]);
  mOutsideThreshold = (float)mEEPROM_OutsideSetpoint - ((float)mEEPROM_OutsideSetpoint * cDetectionSensitivity[mEEPROM_DetectionSensitivityIndex]);
  sprintf(mInsideSensorThreshold, "%04d", (uint16_t)mInsideThreshold);
  sprintf(mOutsideSensorThreshold, "%04d", (uint16_t) mOutsideThreshold);
  Serial.print("mInsideSensorThreshold=");Serial.println(mInsideSensorThreshold);
  Serial.print("mOutsideSensorThreshold=");Serial.println(mOutsideSensorThreshold);
  if (!mInsideSensorBootOk) {
    sprintf(mInsideSensorThreshold, " INV");
    sprintf(mInsideSensorSetpoint, " INV");
    sprintf(mInsideSensorValue, " INV");
  }
  if (!mOutsideSensorBootOk) {
    sprintf(mOutsideSensorThreshold, " INV");
    sprintf(mOutsideSensorSetpoint, " INV");
    sprintf(mOutsideSensorValue, " INV");
  }
  LcdUpdateSensorValuesOnly.start();
  LcdUpdate.start();
  updateLcd();
}

void loop() {
  mTimeNow = millis();

  readSensors(); // Switch, button, lasers
  evaluateData();
  evaluateDoorState();
  evaluateBuzzer();
  
  readRtc(); // RTC

  // Update LCD screen
  if (LcdUpdate.expired()) {
    LcdUpdate.stop();
    updateLcd();
    LcdUpdate.start();
  }

  if (LcdUpdateSensorValuesOnly.expired()) {
    LcdUpdateSensorValuesOnly.stop();
    if (mCurrentMode == MODE_MAIN) {
      updateLcdMainscreenSensors();
    }
    LcdUpdateSensorValuesOnly.start();
  }
}

void evaluateDoorState() {

  // If bottom button is pressed
  if (BottomButtonPress.expired()) {
    // stop using buzzer every time button is pressed. its annoying.
    //BuzzerButton.limitMillis(BUZZER_BUTTON_ON_TIME_CONFIG_MS);
    //BuzzerButton.start();
    //if (mEEPROM_UseBuzzer) turnOnBuzzer();
    if (mCurrentMode == MODE_MAIN) {
      if (mCurrentDoorState == STATE_DOOR_CLOSED) {
        Serial.println("Opening Door due to button press.");
        BottomButtonPress.stop();
        if (mEEPROM_UseBuzzer) {
          BuzzerButton.start();
          turnOnBuzzer();
          BuzzerButton.limitMillis(BUZZER_BUTTON_ON_TIME_DOOR_MS);
        }
        moveDoor(OPEN);
        DoorOpenedFromButtonPress.start();
      }
      if (!mEEPROM_UnlockInMorning) {
        bUnlockedInMorning = true;
      }
    }
    else if (mCurrentMode == MODE_CALIBRATE) {
      setDistanceSetpoints(INSIDE);
      setDistanceSetpoints(OUTSIDE);
    }
    else if (mCurrentMode == MODE_CONFIG_0) {
      // change open time
      mEEPROM_DoorOpenHourIndex++;
      if (mEEPROM_DoorOpenHourIndex >= cTimeConfigurationsAM_size)
        mEEPROM_DoorOpenHourIndex = 0;
      EEPROM.put(EEPROM_DOOR_OPEN_HOUR_ADDR,mEEPROM_DoorOpenHourIndex);
      Serial.print("Changed mEEPROM_DoorOpenHourIndex=");Serial.println(mEEPROM_DoorOpenHourIndex);
    }
    else if (mCurrentMode == MODE_CONFIG_1) {
      // change close time
      mEEPROM_DoorLockedHourIndex++;
      if (mEEPROM_DoorLockedHourIndex >= cTimeConfigurationsPM_size)
        mEEPROM_DoorLockedHourIndex = 0;
      EEPROM.put(EEPROM_DOOR_LOCKED_HOUR_ADDR,mEEPROM_DoorLockedHourIndex);
      Serial.print("Changed mEEPROM_DoorLockedHourIndex=");Serial.println(mEEPROM_DoorLockedHourIndex);
      //Serial.print("sizeof(cTimeConfigurationsPM)=");Serial.println(sizeof(cTimeConfigurationsPM));
      //Serial.print("sizeof(uint8_t)=");Serial.println(sizeof(uint8_t));
    }
    else if (mCurrentMode == MODE_CONFIG_2) {
      if (mEEPROM_UnlockInMorning) mEEPROM_UnlockInMorning = false;
      else mEEPROM_UnlockInMorning = true;
      EEPROM.put(EEPROM_OPEN_IN_MORNING_ADDR,mEEPROM_UnlockInMorning);
      Serial.print("Changed mEEPROM_UnlockInMorning=");Serial.println(mEEPROM_UnlockInMorning);

    }
    else if (mCurrentMode == MODE_CONFIG_3) {
      mEEPROM_DetectionSensitivityIndex++;
      if (mEEPROM_DetectionSensitivityIndex >= cDetectionSensitivity_size)
        mEEPROM_DetectionSensitivityIndex = 0;
      EEPROM.put(EEPROM_DETECTION_SENSITIVITY_ADDR,mEEPROM_DetectionSensitivityIndex);
      // Update thresholds with new sensitivity
      mInsideThreshold = (float)mEEPROM_InsideSetpoint - ((float)mEEPROM_InsideSetpoint * cDetectionSensitivity[mEEPROM_DetectionSensitivityIndex]);
      mOutsideThreshold = (float)mEEPROM_OutsideSetpoint - ((float)mEEPROM_OutsideSetpoint * cDetectionSensitivity[mEEPROM_DetectionSensitivityIndex]);
      sprintf(mInsideSensorThreshold, "%04d", (uint16_t)mInsideThreshold);
      sprintf(mOutsideSensorThreshold, "%04d", (uint16_t)mOutsideThreshold);
    }
    else if (mCurrentMode == MODE_CONFIG_4) {
      mEEPROM_DetectionSamplingRateIndex++;
      if (mEEPROM_DetectionSamplingRateIndex >= cDetectionSamplingRates_size)
        mEEPROM_DetectionSamplingRateIndex = 0;
      EEPROM.put(EEPROM_DETECTION_SAMPLING_RATE_ADDR,mEEPROM_DetectionSamplingRateIndex);
      MotionDetected.limitMillis(cDetectionSamplingRates[mEEPROM_DetectionSamplingRateIndex]);
    }
    else if (mCurrentMode == MODE_CONFIG_5) {
      if (mEEPROM_UseBuzzer) mEEPROM_UseBuzzer = false;
      else mEEPROM_UseBuzzer = true;
      EEPROM.put(EEPROM_USE_BUZZER_ADDR,mEEPROM_UseBuzzer);
    }
    updateLcd();
  }

  // If motion has been detected continuosly for 200ms or so
  if (MotionDetected.expired()) {
    MotionDetected.stop();
    if (mCurrentDoorState == STATE_DOOR_CLOSED) {
      if (
          mOutsideSensorTriggered || 
          (mInsideSensorTriggered && !mEEPROM_RTC_Enabled) || 
          (mInsideSensorTriggered && bIsDoorAllowedOpen && bUnlockedInMorning)
        ) {
        moveDoor(OPEN);
        if (mOutsideSensorTriggered) Serial.println("Opening Door due to outside sensor");
        if (mInsideSensorTriggered && !mEEPROM_RTC_Enabled) Serial.println("Opening Door due to inside sensor (rtc off)");
        if (mInsideSensorTriggered && bIsDoorAllowedOpen && bUnlockedInMorning) Serial.println("Opening Door due to inside sensor (allowed)");
        DoorOpenedFromMotionDetect.start();
      }
    }
  }

  if (MotionNotDetected.expired()) {
    MotionNotDetected.stop();

    if (mCurrentDoorState == STATE_DOOR_OPEN) {
      bool leaveDoorOpen = mOutsideSensorTriggered || 
            (mInsideSensorTriggered && !mEEPROM_RTC_Enabled) || 
            (mInsideSensorTriggered && bIsDoorAllowedOpen && bUnlockedInMorning);
      if ((!DoorOpenedFromMotionDetect.running() &&
           !DoorOpenedFromButtonPress.running() &&
          !leaveDoorOpen) ) {
        moveDoor(CLOSED);
        Serial.println( " Closing door due to motion not detected");
      }
    }
  }

  if (DoorOpenedFromButtonPress.expired()) {
    DoorOpenedFromButtonPress.stop();
    bool leaveDoorOpen = mOutsideSensorTriggered || 
          (mInsideSensorTriggered && !mEEPROM_RTC_Enabled) || 
          (mInsideSensorTriggered && bIsDoorAllowedOpen && bUnlockedInMorning);
    if ((mCurrentDoorState == STATE_DOOR_OPEN) &&
         !DoorOpenedFromMotionDetect.running() &&
         !leaveDoorOpen) {
      moveDoor(CLOSED);
      Serial.println("Closing Door - done from button press");    
    }
  }

  if (DoorOpenedFromMotionDetect.expired()) {
    DoorOpenedFromMotionDetect.stop();
    bool leaveDoorOpen = mOutsideSensorTriggered || 
          (mInsideSensorTriggered && !mEEPROM_RTC_Enabled) || 
          (mInsideSensorTriggered && bIsDoorAllowedOpen && bUnlockedInMorning);
    if ((mCurrentDoorState == STATE_DOOR_OPEN) &&
         !DoorOpenedFromButtonPress.running() &&
         !leaveDoorOpen) {
      moveDoor(CLOSED);
      Serial.println("Closing Door due to end of motion detect time");
    }
  }
}

void evaluateData() {
  
  if (TopButtonHold.expired()) {
     // Serial.println(" Top button held!");
    mModeChangeTime = millis();
    mCurrentMode++;
    if (mCurrentMode > NUM_MODES) mCurrentMode = MODE_MAIN;
    Serial.print("Top button pressed, moving to mode: ");Serial.println(mCurrentMode);
    updateLcd();
    //if (mEEPROM_UseBuzzer) {
    //  turnOnBuzzer();      
    //  BuzzerButton.limitMillis(BUZZER_BUTTON_ON_TIME_CONFIG_MS);
    //  BuzzerButton.start();
    //}
    TopButtonHold.stop();
  }

  if (mSwitchPosition == LEFT) mEEPROM_RTC_Enabled = true;
  else if (mSwitchPosition == RIGHT) mEEPROM_RTC_Enabled = false;

  //if (mEEPROM_RTC_Enabled) {
  uint8_t amHour = cTimeConfigurationsAM[mEEPROM_DoorOpenHourIndex][0];
  uint8_t pmHour = cTimeConfigurationsPM[mEEPROM_DoorLockedHourIndex][0]+12; // compensate for 24H time
  uint8_t amMinutes = cTimeConfigurationsAM[mEEPROM_DoorOpenHourIndex][1];
  uint8_t pmMinutes = cTimeConfigurationsPM[mEEPROM_DoorLockedHourIndex][1];
  // if ((mCurrentHour >= LOCK_DOOR_HOUR) || (mCurrentHour < OPEN_DOOR_HOUR)) {
  //  Serial.print(" mCurrentHour=");Serial.println(mCurrentHour);
  //  Serial.print(" mCurrentMinute=");Serial.println(mCurrentMinute);
  //  Serial.print(" pmHour=");Serial.println(pmHour);
  //  Serial.print(" pmMinutes=");Serial.println(pmMinutes);

  // Decide whether we are in DAY or NIGHT
  if (((mCurrentHour == pmHour) && (mCurrentMinute > pmMinutes)) || (mCurrentHour > pmHour) || 
      ((mCurrentHour == amHour) && (mCurrentMinute < amMinutes)) || (mCurrentHour < amHour) ) {
    bIsDoorAllowedOpen = false;
    if ((mCurrentHour >= pmHour) || (mCurrentHour < RTC_BUTTON_UNLOCK_EARLIEST_AM_HOUR)) { //only reset unlock trigger at night / before 4AM
      bUnlockedInMorning = false;
    }
    if (DEBUG) {Serial.print("Set bIsDoorAllowedOpen=");Serial.println(bIsDoorAllowedOpen);}
    if (DEBUG) {Serial.print("Set bUnlockedInMorning=");Serial.println(bUnlockedInMorning);}

  }
  else if (((mCurrentHour == pmHour) && (mCurrentMinute <= pmMinutes)) ||  (mCurrentHour < pmHour) || 
            ((mCurrentHour == amHour) && (mCurrentMinute >= amMinutes)) || (mCurrentHour > amHour) ) {
    bIsDoorAllowedOpen = true;
    if (mEEPROM_UnlockInMorning) { // otherwise only unlock when button is pressed
      if (!bUnlockedInMorning) {
        if (mEEPROM_UseBuzzer) {          
          turnOnBuzzer();
          BuzzerButton.limitMillis(BUZZER_BUTTON_ON_TIME_DOOR_MS);
          BuzzerButton.start();
        }
      }
      bUnlockedInMorning = true;        
    }
    if (DEBUG) {Serial.print("Set bIsDoorAllowedOpen=");Serial.println(bIsDoorAllowedOpen);}
    if (DEBUG) {Serial.print("Set bUnlockedInMorning=");Serial.println(bUnlockedInMorning);}
  }
  //}
  //else {
  if (mEEPROM_RTC_Enabled) { // override above    
    bIsDoorAllowedOpen = true;
  }

  // Decide if inside / outside sensors have reached threshold
  // mEEPROM_DetectionSensitivityIndex is 1-10 --> correspond to 5% to 50% change of setpoint to trigger
  //float insideThresholdPercentage = mEEPROM_DetectionSensitivityIndex / 20.0f;  // ( sensitivity * 5 / 100 )
 // float insideThreshold = (float)mEEPROM_InsideSetpoint - ((float)mEEPROM_InsideSetpoint * cDetectionSensitivity[mEEPROM_DetectionSensitivityIndex]);
  //Serial.print("Inside Threshold: "); Serial.print(cDetectionSensitivity[mEEPROM_DetectionSensitivityIndex]); Serial.print("%  --  ");Serial.println(insideThreshold);
  if ((float)mLaserDistanceInside < mInsideThreshold) {
    mInsideSensorTriggered = true;
    mMotionDetected = true;
    
    if (!MotionDetected.running()) {
      MotionDetected.limitMillis(cDetectionSamplingRates[mEEPROM_DetectionSamplingRateIndex]/2);
      MotionDetected.start();
    }
    mTimeOfInsideMotionDetected = millis();
    //turnOnLED(BLUE);
    //if (!BlinkGreenLED.running()) {
    //  BlinkGreenLED.start();
    //  turnOnLED(GREEN);
    //  mStateOfGreenLED = ON;
    //}
    // changed GREEN inside LED to YELLOW as inside sensor LED
    if (!BlinkYellowLED.running()) {
      BlinkYellowLED.start();
      turnOnLED(YELLOW);
      mStateOfYellowLED = ON;
    }
  }
  else {
    mInsideSensorTriggered = false;
    //turnOffLED(GREEN);
    //mStateOfGreenLED = OFF;
    //BlinkGreenLED.stop();
    turnOffLED(YELLOW);
    mStateOfYellowLED = OFF;
    BlinkYellowLED.stop();
  }
  if ((float)mLaserDistanceOutside < mOutsideThreshold) {
    mOutsideSensorTriggered = true;
    mMotionDetected = true;
    mTimeOfOutsideMotionDetected = millis();
    //turnOnLED(BLUE);
    if (!MotionDetected.running()) {
      MotionDetected.limitMillis(cDetectionSamplingRates[mEEPROM_DetectionSamplingRateIndex]);
      MotionDetected.start();
    }
    // changed YELLOW outside LED to BLUE as outside sensor LED
    //if (!BlinkYellowLED.running()) {
    //  BlinkYellowLED.start();
    //  turnOnLED(YELLOW);
    //  mStateOfYellowLED = ON;
    //}
    if (!BlinkBlueLED.running()) {
      BlinkBlueLED.start();
      turnOnLED(BLUE);
      mStateOfBlueLED = ON;
    }
  }
  else {
    mOutsideSensorTriggered = false;
    //turnOffLED(YELLOW);
    //mStateOfYellowLED = OFF;
    //BlinkYellowLED.stop();
    turnOffLED(BLUE);
    mStateOfBlueLED = OFF;
    BlinkBlueLED.stop();
  }

  if (!mOutsideSensorTriggered && !mInsideSensorTriggered) {
    MotionDetected.stop();
    mMotionDetected = false;
    //turnOffLED(BLUE);
    if (!MotionNotDetected.running()) MotionNotDetected.start();
  }
  // this blink stuff needs to be above the stuff below
  if (BlinkGreenLED.expired()) {
    BlinkGreenLED.start();
    if (mStateOfGreenLED == OFF) {
      turnOnLED(GREEN);
    }
    else {
      turnOffLED(GREEN);
    }
    mStateOfGreenLED = !mStateOfGreenLED;
  }
  
  if (BlinkYellowLED.expired()) {
    BlinkYellowLED.start();
    if (mStateOfYellowLED == OFF) {
      turnOnLED(YELLOW);
    }
    else {
      turnOffLED(YELLOW);
    }
    mStateOfYellowLED = !mStateOfYellowLED;
  }

  if (BlinkBlueLED.expired()) {
    BlinkBlueLED.start();
    if (mStateOfBlueLED == OFF) {
      turnOnLED(BLUE);
    }
    else {
      turnOffLED(BLUE);
    }
    mStateOfBlueLED = !mStateOfBlueLED;
  }

  if (!mEEPROM_RTC_Enabled || (bIsDoorAllowedOpen && bUnlockedInMorning)) {
    //turnOffLED(BLUE);
    turnOnLED(GREEN);
  }
  else {
    //turnOnLED(BLUE);
    turnOffLED(GREEN);
  }

  // if outside sensor was triggered last, turn on Yellow LED
  if ((mTimeOfOutsideMotionDetected != 0) && (mTimeOfInsideMotionDetected != 0) && !mMotionDetected) {
   // BlinkGreenLED.stop();
  //  BlinkYellowLED.stop();
    if (mTimeOfOutsideMotionDetected > mTimeOfInsideMotionDetected) {
      //turnOnLED(YELLOW);
      //turnOffLED(GREEN);
      turnOffLED(YELLOW);
      turnOnLED(BLUE);
    }
    else { // inside sensor triggered last
      //turnOffLED(YELLOW);
      //turnOnLED(GREEN);
      turnOnLED(YELLOW);
      turnOffLED(BLUE);
    }
  }

  // if both sensors are within valid thresholds, reset our reboot counter so we can auto-reboot when they stop working
  if ( (mLaserDistanceInside != mLaserDistanceOutside) &&
        (mLaserDistanceInside > 0) &&
        (mLaserDistanceInside < 2000) &&
        (mLaserDistanceOutside > 0) &&
        (mLaserDistanceOutside < 2000)
        ) {
    if (mEEPROM_RebootCounter != 0) {
      mEEPROM_RebootCounter = 0;
      EEPROM.put(EEPROM_REBOOT_COUNTER_ADDR,mEEPROM_RebootCounter);
      Serial.print("Reset mEEPROM_RebootCounter=");Serial.println(mEEPROM_RebootCounter);
    }      
  }


} // end evaluateData

void evaluateBuzzer() {
  // 
  if (!mEEPROM_UseBuzzer) {
    turnOffBuzzer();
    BuzzerAlarm.stop();
    return;
  }

  if (mCurrentMode != MODE_MAIN) {
    BuzzerAlarm.stop();
  }
  // Check sensors to see if outdoor sensor may have become invalid- not sure why this happens. 
  // Reboot fixes this problem. Buzzer is to alert someone to reboot
  // Check if indoor reading is the same as the outdoor, x times in a row?
  //Serial.print("LaserInside:");Serial.println(mLaserDistanceInside);
  //Serial.print("LaserOutside:");Serial.println(mLaserDistanceOutside);

  if (        (mLaserDistanceInside  == mLaserDistanceOutside)  // outside sensor unplugged or stopped functioning
          ||  (mLaserDistanceInside  == INVALID_READING) 
          ||  (mLaserDistanceOutside == INVALID_READING)
          ||  (mLaserDistanceInside  == -1)
          ||  (mLaserDistanceInside  == -1)                  ){
    //Serial.print("ALARM CONDITION REACHED. mEEPROM_UseBuzzer=");Serial.println(mEEPROM_UseBuzzer);
    //Serial.print("ALARM CONDITION REACHED. limitMillis=");Serial.println(BuzzerAlarm.limitMillis());

    if (!BuzzerAlarm.running()) BuzzerAlarm.start();
    if (BuzzerAlarm.expired()) {
      //Serial.println("Buzzer alarm expired");
      BuzzerAlarm.stop();
      // trigger reboot if outside sensor stopped working, but only reboot once before giving up
      if ( (mLaserDistanceInside == mLaserDistanceOutside) &&
           (mLaserDistanceInside > 0) &&
           (mLaserDistanceInside < 2000) &&
           (mEEPROM_RebootCounter == 0)
           ) {
        mEEPROM_RebootCounter++;
        EEPROM.put(EEPROM_REBOOT_COUNTER_ADDR,mEEPROM_RebootCounter);
        Serial.print("Incremented mEEPROM_RebootCounter=");Serial.println(mEEPROM_RebootCounter);
        if (mCurrentDoorState == STATE_DOOR_OPEN) {
          moveDoor(CLOSED);
        }
        turnOnBuzzer();
        delay(1000);
        turnOffBuzzer();
        delay(500);
        turnOnBuzzer();
        delay(1000);
        turnOffBuzzer();
        delay(500);
        turnOnBuzzer();
        delay(1000);
        turnOffBuzzer();
        delay(500);
        turnOnBuzzer();
        delay(1000);
        turnOffBuzzer();
        delay(500);
        turnOnBuzzer();
        delay(1000);
        turnOffBuzzer();
        delay(500);
        resetFunc(); // reset arduino
      }

      if (BuzzerAlarm.limitMillis() == BUZZER_ALARM_ON_TIME_MS) {
        BuzzerAlarm.limitMillis(BUZZER_ALARM_OFF_TIME_MS);
        turnOffBuzzer();
      }
      else if (BuzzerAlarm.limitMillis() == BUZZER_ALARM_OFF_TIME_MS) {
        BuzzerAlarm.limitMillis(BUZZER_ALARM_ON_TIME_MS);
        turnOnBuzzer();
      }
      BuzzerAlarm.start();
    }    
  }
  else {
    //Serial.print("ALARM CONDITION EXPIREED");
    BuzzerAlarm.stop();
  }
  
  // Turn off buzzer sound from button push
  if (BuzzerButton.expired()) {
    turnOffBuzzer();
    BuzzerButton.stop();
  }
  
  

}


// this can be used as a callback (?)
void updateLcd() {
  TCA9548A(RTC_DISPLAY_CHAN);  

  if (mCurrentMode == MODE_MAIN) {
    updateLcdMainscreen();
  //  if (mSwitchPosition == LEFT) updateLcdMainscreen();
  //  else if (mSwitchPosition == RIGHT) updateLcdCalibrate();
  }
  else if (mCurrentMode == MODE_CALIBRATE) {
    updateLcdCalibrate();
  }
  else {
    updateLcdConfiguration();
  }
  /*
  else if (mCurrentMode == MODE_CONFIG_0) {
    updateLcdConfiguration();
  }
  else if (mCurrentMode == MODE_CONFIG_1) {
    updateLcdConfiguration1();
  }
  else if (mCurrentMode == MODE_CONFIG_2) {
    updateLcdConfiguration2();
  }
  */
 // delay(100);
}



/*
  EEPROM.get(EEPROM_DOOR_LOCKED_HOUR_ADDR, mEEPROM_DoorLockedHourIndex);
  EEPROM.get(EEPROM_DOOR_OPEN_HOUR_ADDR, mEEPROM_DoorOpenHourIndex);
  EEPROM.get(EEPROM_OPEN_IN_MORNING_ADDR, mEEPROM_UnlockInMorning);
  EEPROM.get(EEPROM_RTC_ENABLED_ADDR, mEEPROM_RTC_Enabled);
*/

// Update LCD for configuration screen
void updateLcdConfiguration() {
  lcd.setCursor(0,0); // clear 1st row
  lcd.print("                    ");
  lcd.setCursor(0,1); // clear 2nd row
  lcd.print("                    ");
  lcd.setCursor(0,2); // clear 3rd row
  lcd.print("                    ");
  lcd.setCursor(0,3); // clear 4th row
  lcd.print("                    ");

  if (mCurrentMode == MODE_CONFIG_0) {
    lcd.setCursor(1,1);
    lcd.print("Unlock Time:");
    lcd.setCursor(1,2);
    lcd.print(cTimeConfigurationsAM[mEEPROM_DoorOpenHourIndex][0]);
    lcd.setCursor(3,2);
    lcd.print(":");
    lcd.setCursor(4,2);
    lcd.print(cTimeConfigurationsAM[mEEPROM_DoorOpenHourIndex][1]);
    lcd.setCursor(6,2);
    lcd.print(" am");
    lcd.setCursor(0,3); // clear 4th row
    lcd.print("                    ");
  }
  else if (mCurrentMode == MODE_CONFIG_1) {
    lcd.setCursor(1,1);
    lcd.print("Door Locked Time:");
    lcd.setCursor(1,2);
    lcd.print(cTimeConfigurationsPM[mEEPROM_DoorLockedHourIndex][0]);
    lcd.setCursor(3,2);
    lcd.print(":");
    lcd.setCursor(4,2);
    lcd.print(cTimeConfigurationsPM[mEEPROM_DoorLockedHourIndex][1]);
    lcd.setCursor(6,2);
    lcd.print("pm");
    lcd.setCursor(0,3); // clear 4th row
    lcd.print("                    ");
  }
  else if (mCurrentMode == MODE_CONFIG_2) {

    lcd.setCursor(0,1);
    lcd.print("Unlock In Morning?");
    lcd.setCursor(3,2);
    if (mEEPROM_UnlockInMorning)
      lcd.print("YES");
    else
      lcd.print(" NO");
    lcd.setCursor(0,3); // clear 4th row
    lcd.print("                    ");
  }
  else if (mCurrentMode == MODE_CONFIG_3) {
    lcd.setCursor(0,0);
    lcd.print("Detect Sensitivity");
    lcd.setCursor(3,1);
    lcd.print(cDetectionSensitivity[mEEPROM_DetectionSensitivityIndex]);
    lcd.setCursor(2,2);
    lcd.print("Threshholds");
    lcd.setCursor(0,3);
    lcd.print("In:");
    lcd.setCursor(3,3);
    lcd.print(String(mInsideSensorThreshold));
    lcd.setCursor(9,3);
    lcd.print(" Out:");
    lcd.setCursor(14,3); 
    lcd.print(String(mOutsideSensorThreshold));

  }
  else if (mCurrentMode == MODE_CONFIG_4) {
    lcd.setCursor(1,0);
    lcd.print("Detection");
    lcd.setCursor(3,1);
    lcd.print("Sampling Rate");
    lcd.setCursor(9,2);
    lcd.print(cDetectionSamplingRates[mEEPROM_DetectionSamplingRateIndex]);
    lcd.setCursor(0,3); // clear 4th row
    lcd.print("                    ");
  }
  else if (mCurrentMode == MODE_CONFIG_5) {
    lcd.setCursor(2,1);
    lcd.print("Buzzer Enabled?");
    lcd.setCursor(3,2);
    if (mEEPROM_UseBuzzer)
      lcd.print("YES");
    else
      lcd.print(" NO");
    lcd.setCursor(0,3); // clear 4th row
    lcd.print("                    ");
  }

}

// Update LCD for secondary screen for calibration (switch is right)
void updateLcdCalibrate() {

  //lcd.clear();
  lcd.setCursor(0,0); // clear 1st row
  lcd.print("                    ");
  lcd.setCursor(0,1); // clear 2nd row
  lcd.print("                    ");
  lcd.setCursor(0,2); // clear 3rd row
  lcd.print("                    ");
  lcd.setCursor(0,3); // clear 4th row
  lcd.print("                    ");

  lcd.setCursor(0,0);
  lcd.print("Push Buttn to Calibr");

  // Print sensor data regardless of state
  lcd.setCursor(0,1);
  lcd.print(" Inside  |  Outside ");
  lcd.setCursor(0,2);
  lcd.print("Set |Curr|Set |Curr ");
  lcd.setCursor(0,3); 
  lcd.print(String(mInsideSensorSetpoint));
  lcd.setCursor(5,3); 
  lcd.print(String(mInsideSensorValue));
  lcd.setCursor(10,3); 
  lcd.print(String(mOutsideSensorSetpoint));
  lcd.setCursor(15,3); 
  lcd.print(String(mOutsideSensorValue));

}

void updateLcdMoveDoor() {
  if (mCurrentMode == MODE_MAIN) {
    TCA9548A(RTC_DISPLAY_CHAN);  
    lcd.setCursor(14,0);
    if (mCurrentDoorState == STATE_DOOR_CLOSED) {
      lcd.print("CLOSED");
    }
    else {
      lcd.print(" OPEN ");
    }
  }
}

// Update LCD screen for main screen (switch is left)
void updateLcdMainscreen() {
  //Serial.println(" UpdatelcdMainscreen()");
  //lcd.clear();
  lcd.setCursor(0,0); // clear 3rd row
  lcd.print("                    ");
  lcd.setCursor(0,1); // clear 3rd row
  lcd.print("                    ");
  lcd.setCursor(0,2); // clear 3rd row
  lcd.print("                    ");
  // Show RTC: OPEN, RTC:CLOSED, RTC: OFF
  //  Inside: Curr | Setp
  // Outside: Curr | Setp
  //       Time
 // if (mCurrentDoorState == STATE_DOOR_CLOSED) {
  lcd.setCursor(0,0);  
  if (mEEPROM_RTC_Enabled) {
    if (bIsDoorAllowedOpen && bUnlockedInMorning) {
      lcd.print("RTC: UNLOCKED");
    }
    else if (!bUnlockedInMorning) {
      //if (mCurrentHour >= pmHour) {
      //  lcd.print("RTC: PM-LOCK ");
      //}
      //else {
        lcd.print("RTC: AM-LOCK ");
      //}
    }
    else {
      lcd.print("RTC: RTC-LOCK");
    }
  }
  else {
    lcd.print("RTC: DISABLED");
  }
  lcd.setCursor(14,0);
  if (mCurrentDoorState == STATE_DOOR_CLOSED) {
    lcd.print("CLOSED");
  }
  else {
    lcd.print(" OPEN ");
  }


  // Print sensor data regardless of state
  lcd.setCursor(0,1);
  lcd.print(" Inside: ");
  lcd.setCursor(9,1); 
  lcd.print(String(mInsideSensorValue));

  lcd.setCursor(14,1);
  lcd.print("|");
  lcd.setCursor(16,1); 
  lcd.print(String(mInsideSensorThreshold));

  lcd.setCursor(0,2); 
  lcd.print("Outside: ");
  lcd.setCursor(9,2); 
  lcd.print(String(mOutsideSensorValue));

  lcd.setCursor(14,2);
  lcd.print("|");
  lcd.setCursor(16,2); 
  lcd.print(String(mOutsideSensorThreshold));
  
  // Update current time
  updateLcdMainscreenTime();

}

// Clears and prints only the sensor data on LCD
void updateLcdMainscreenSensors() {
  lcd.setCursor(9,1); 
  lcd.print(String(mInsideSensorValue));
  lcd.setCursor(9,2); 
  lcd.print(String(mOutsideSensorValue));
}

// Prints Current time on 3rd row of LCD
void updateLcdMainscreenTime() {
   if (DEBUG) {
    Serial.println("updateLcdMainscreenTime() start");
    Serial.print("mCurDate: "); Serial.println(mCurDate);
    Serial.print("mCurMonth: "); Serial.println(mCurMonth);
    Serial.print("mCurMinute: "); Serial.println(mCurMinute);
    Serial.print("mCurHour: "); Serial.println(mCurHour);
    Serial.println("updateLcdMainscreenTime() end");
   }
  // This should only be called from inside updateLcdMainscreen(), where the mux has already selected this channel
  //TCA9548A(RTC_DISPLAY_CHAN);
  
  lcd.setCursor(0,3); // clear 3rd row
  lcd.print("                    ");

  // print DATE
  lcd.setCursor(0,3);
  lcd.print(String(mCurMonth));
  lcd.setCursor(2,3);
  lcd.print("/");
  lcd.setCursor(3,3);
  lcd.print(String(mCurDate));
  lcd.setCursor(5,3);
  lcd.print("/");
  lcd.setCursor(6,3);
  lcd.print(realTimeClock.getYear());

  // PRINT TIME
  lcd.setCursor(9,3);

  if (mCurrentHour > 12) {
    if ((mCurrentHour-12) > 9) {
      lcd.print(mCurrentHour-12);
    }
    else {
      lcd.print("0");
      lcd.setCursor(10,3);
      lcd.print(mCurrentHour-12);
    }
  }
  else if (mCurrentHour > 9) {
    lcd.print(mCurrentHour);
  }
  else {
    lcd.print("0");
    lcd.setCursor(10,3);
    lcd.print(mCurrentHour);
  }
  lcd.setCursor(11,3);
  lcd.print(":");
  lcd.setCursor(12,3);
  lcd.print(String(mCurMinute));

  lcd.setCursor(15,3);
  if (mCurrentHour > 11) {
    lcd.print("PM");
  }
  else {
    lcd.print("AM");
  }
  lcd.setCursor(17,3);
  lcd.print("  ");
}


void setDistanceSetpoints(uint8_t inLaserChan) {
  if (inLaserChan == 0) {
    mEEPROM_InsideSetpoint = mLaserDistanceInside;
    EEPROM.put(EEPROM_INSIDE_SETPOINT_ADDR,mEEPROM_InsideSetpoint);
    sprintf(mInsideSensorSetpoint, "%04d", (unsigned int)mEEPROM_InsideSetpoint);
  }
  else if (inLaserChan == 1) {
    mEEPROM_OutsideSetpoint = mLaserDistanceOutside;
    EEPROM.put(EEPROM_OUTSIDE_SETPOINT_ADDR,mEEPROM_OutsideSetpoint);
    sprintf(mOutsideSensorSetpoint, "%04d", (unsigned int)mEEPROM_OutsideSetpoint);
  }

  mInsideThreshold = (float)mEEPROM_InsideSetpoint - ((float)mEEPROM_InsideSetpoint * cDetectionSensitivity[mEEPROM_DetectionSensitivityIndex]);
  mOutsideThreshold = (float)mEEPROM_OutsideSetpoint - ((float)mEEPROM_OutsideSetpoint * cDetectionSensitivity[mEEPROM_DetectionSensitivityIndex]);

  sprintf(mInsideSensorThreshold, "%04d", (uint16_t)mInsideThreshold);
  sprintf(mOutsideSensorThreshold, "%04d", (uint16_t)mOutsideThreshold);

  Serial.print("setDistanceSetpoints- mEEPROM_InsideSetpoint=");Serial.print(mEEPROM_InsideSetpoint);
  Serial.print("  mEEPROM_OutsideSetpoint=");Serial.print(mEEPROM_OutsideSetpoint);
  Serial.println("");
  Serial.print("  mInsideThreshold=");Serial.print(mInsideThreshold);
  Serial.print("  mOutsideThreshold=");Serial.print(mOutsideThreshold);
  Serial.println("");

}

void readSensors() {
  // if button has not been pressed
  if (digitalRead(USER_BUTTON_0) == HIGH) {
    mButtonPressedBottom = false;
    BottomButtonHold.stop();
    BottomButtonPress.stop();
  }
  else { // if button is pressed
    mButtonPressedBottom = true;
    if (!BottomButtonHold.running()) BottomButtonHold.start();
    if (!BottomButtonPress.running()) BottomButtonPress.start();
  }

  if (digitalRead(USER_BUTTON_1) == HIGH) {
    mButtonPressedTop = false;
    TopButtonHold.stop();
  }
  else { // if button is pressed
    mButtonPressedTop = true;
    if (!TopButtonHold.running()) {
      TopButtonHold.start();
    }
  }

  mSwitchPosition = (digitalRead(USER_SWITCH) == HIGH) ? LEFT : RIGHT;
  //mButtonPressed = (digitalRead(USER_BUTTON) == HIGH) ? false : true;

  if (mInsideSensorBootOk) {
    mLaserDistanceInside = getLaserReading(INSIDE);
    sprintf(mInsideSensorValue, "%04d", (unsigned int)mLaserDistanceInside);
  }
  if (mOutsideSensorBootOk) {
    mLaserDistanceOutside = getLaserReading(OUTSIDE);
    sprintf(mOutsideSensorValue, "%04d", (unsigned int)mLaserDistanceOutside);
  }
  // if both readings are the same, it means the outside sensor is not connected and we reading the inside sensor twice
  //if (mLaserDistanceOutside == mLaserDistanceInside) {
  //  mLaserDistanceOutside = 0xFFFF;    
  //}

  if (DEBUG) {
    Serial.println("readSensors() start");
    Serial.print("mSwitchPositional: "); Serial.println(mSwitchPosition);
    Serial.print("mButtonPressedTop: "); Serial.println(mButtonPressedTop);
    Serial.print("mButtonPressedBottom: "); Serial.println(mButtonPressedBottom);
    Serial.print("mInsideSensorValue: "); Serial.println(mInsideSensorValue);
    Serial.print("mOutsideSensorValue: "); Serial.println(mOutsideSensorValue);
    Serial.println("readSensors() end");
  }

}

// Spin motor
void moveDoor(bool inOpenDoor) {
  if (inOpenDoor) {
    Serial.println("moveDoor open");
    digitalWrite(STEPPER_DIR_PIN, HIGH);
    mCurrentDoorState = STATE_DOOR_OPEN;
  }
  else {
    Serial.println("moveDoor closed");
    digitalWrite(STEPPER_DIR_PIN, LOW);
    mCurrentDoorState = STATE_DOOR_CLOSED;
  }
  updateLcdMoveDoor();
  for (uint16_t i=0; i<STEPPER_NUM_STEPS_TO_OPEN_DOOR; i++) {
    delayMicroseconds(STEPPER_STEP_LENGTH_US);
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(STEPPER_STEP_LENGTH_US);
    digitalWrite(STEPPER_STEP_PIN, LOW);
  }
  // reset DIR pin or else arduino freaks out. Not sure why?
  digitalWrite(STEPPER_DIR_PIN, LOW);

}


uint16_t getLaserReading(uint8_t inLaserChan) {
  // select i2c mux channel
  if (inLaserChan == INSIDE) TCA9548A(LASER_CHAN_0);
  else TCA9548A(LASER_CHAN_1);
  laserSensor.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    return measure.RangeMilliMeter;
  } 
  else {
    return INVALID_READING;
  }

}

void readRtc() {
  TCA9548A(RTC_DISPLAY_CHAN);  // careful!  Only call this function after LCD update so mux channel is already selected
  mCurrentHour = realTimeClock.getHour(mClockFlag12H, mClockFlagPM);
  mCurrentMinute = realTimeClock.getMinute();
  
  sprintf(mCurDate, "%02d", unsigned(realTimeClock.getDate()));
  sprintf(mCurMonth, "%02d", unsigned(realTimeClock.getMonth(mClockCentury)));
  sprintf(mCurMinute, "%02d", unsigned(mCurrentMinute));
  sprintf(mCurHour, "%02d", unsigned(mCurrentHour));

  if (DEBUG) {
    Serial.print("readRtc(): mCurrentHour=");Serial.println(mCurrentHour);
    Serial.print("readRtc(): mCurrentMinute=");Serial.println(mCurrentMinute);    
  }

}

// Select I2C BUS
void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  delay(MUX_DELAY_MS);
  //Serial.print(bus);
}

void turnOnLED(uint8_t inLED) {
  digitalWrite(inLED, HIGH);
}
void turnOffLED(uint8_t inLED) {
  digitalWrite(inLED, LOW);
}

void turnOnBuzzer() {
  digitalWrite(BUZZER_PIN, LOW);
}
void turnOffBuzzer() {
 // Serial.println(    "Turn off buzzer");
  digitalWrite(BUZZER_PIN, HIGH);
}


