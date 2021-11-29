#include <Arduino.h>
#include <Wire.h>
#include <sim800.h>
//#include <WiFi.h>
#include <gprs.h>
#include <Esp.h>

#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Run 'make menuconfig' to enable!
#endif


// AceButton
#include <AceButton.h>

// Rotary encoder
#include <SimpleRotary.h>

// LCD Display
#include <LiquidCrystal_I2C.h>

// Flash memory
#include <Preferences.h>

// BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)

// Free I/O pins
// 12 14 16 18 19 25 32 33 35

// Input pins
//#define BTN_PIN 16 NOT IN USE
#define SIDEBTN_PIN 14 // can be set as external wake up source
#define ROTBTN_PIN 2
#define ROTA 0
#define ROTB 15
#define BATT_V 13

#define T_CALL

// SIM800 communication
#define UART_TX                     27
#define UART_RX                     26
#define SIMCARD_RST                 5
#define SIMCARD_PWKEY               4
#define SIM800_POWER_ON             23

#define UART_BANUD_RATE             9600

#define I2C_SDA                     21 //YELLOW
#define I2C_SCL                     22 //GREEN

#define IP5306_ADDR                 0X75
#define IP5306_REG_SYS_CTL0         0x00

// Hardware serial (for SIM800)
HardwareSerial hwSerial(1);
GPRS gprs(hwSerial, SIMCARD_PWKEY, SIMCARD_RST, SIM800_POWER_ON);

BluetoothSerial SerialBT;

// LCD Display
LiquidCrystal_I2C lcd(0x27, 16, 2);
// Flash memory
Preferences preferences;
// BME280
Adafruit_BME280 bme;

// Rotary encoder
SimpleRotary rotEnc(ROTA, ROTB, ROTBTN_PIN);
// AceButton
using namespace ace_button;
//AceButton button(BTN_PIN); NOT IN USE
AceButton sideButton(SIDEBTN_PIN);
AceButton rotButton(ROTBTN_PIN);
void handleEvent(AceButton*, uint8_t, uint8_t);

int NUMBER_AMOUNT = 36;
String names[250];
String numbers[250];
int inCallVol = 15;
int ringerVol = 30;

bool setPowerBoostKeepOn(int en) {
    Wire.beginTransmission(IP5306_ADDR);
    Wire.write(IP5306_REG_SYS_CTL0);
    if (en)
        Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
    else
        Wire.write(0x35); // 0x37 is default reg value
    return Wire.endTransmission() == 0;
}

#pragma region SerialDebug
bool debug = true;
bool debugBattery = true;
void SerialPrintLine(String t, bool force = false) {
  if (debug || force) {
    Serial.println(t);
    SerialBT.println(t);
  }
}

void SerialPrint(String t, bool force = false) {
  if (debug || force) {
    Serial.print(t);
    SerialBT.print(t);
  }
}

bool SerialAvailable() {
  if (debug) {
    return Serial.available();
  }
  return false;
}
#pragma endregion SerialDebug

// TODO:
//  - add mute control AT+CMUT (0 mute off, 1 mute on)
//  - implement jamming detection AT+SJDR
//  - compensate time request delay
// Intresting:
//  - check out antenna detection AT+CANT
//  - check out AT+CNETSCAN
//  - check out AT+CMEDPLAY
//  - roaming detection maybe daily AT+CROAMING
//  - debug speaker AT+SIMTONEX

void setup() {
  // INIT: Turn off WiFi and Bluetooth
  Wire.begin(I2C_SDA, I2C_SCL);
  //WiFi.mode(WIFI_OFF);
  //btStop();

  // INIT: Start LCD
  lcd.begin();
  lcd.backlight();
  WriteText("Booting . . .", "Soyuz V1.1", 2000);
  
  Serial.begin(9600);
  SerialBT.begin("Project Soyuz V1.1");
  SerialBT.enableSSP();
  SerialPrintLine("STARTING", true); // Debug

  // INIT: Start comms with flash memory
  preferences.begin("PHONE", false);
  int dInt = preferences.getInt("DEBUG", 1);
  if (dInt == 1) debug = true;
  else if (dInt == 0) debug = false;
  int dbInt = preferences.getInt("DEBUGBAT", 1);
  if (dbInt == 1) debugBattery = true;
  else if (dbInt == 0) debugBattery = false;


  int cVol = preferences.getInt("CVOL", inCallVol);
  int rVol = preferences.getInt("RVOL", ringerVol);
  inCallVol = cVol;
  ringerVol = rVol;

  // INIT: Setup input via AceButton
  pinMode(SIDEBTN_PIN, INPUT_PULLUP);
  pinMode(ROTBTN_PIN, INPUT_PULLUP);
  ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  //buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  //buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  //buttonConfig->setFeature(ButtonConfig::kFeatureRepeatPress);
  
  #if defined(T_CALL)
    bool   isOk = setPowerBoostKeepOn(1);
    String info = "IP5306 KeepOn " + String((isOk ? "PASS" : "FAIL"));
    SerialPrintLine(info); // Debug
    WriteText("Booting . . .", String((isOk ? "IP5306 PASS" : "IP5306 FAIL")), 500);
  #endif

  hwSerial.begin(UART_BANUD_RATE, SERIAL_8N1, UART_RX, UART_TX);
  
  // INIT: Start hardware initialization
  WriteText("Booting . . .", "SIM800 TEST", 500);
  if (gprs.preInit()) {
      SerialPrintLine("SIM800 Begin PASS"); // Debug
      WriteText("Booting . . .", "SIM800 PASS", 500);
  } else {
      SerialPrintLine("SIM800 Begin FAIL"); // Debug
      WriteText("Booting . . .", "SIM800 FAIL", 500);
  }

  SerialPrintLine("Test motor ...", true); // Debug
  WriteText("Booting . . .", "TEST MOTOR ...", 500);
  int i = 3;
  while (i--) {
      hwSerial.print("AT+SPWM=0,1000,1\r\n"); // PWM for buzzer
      delay(2000);
      hwSerial.print("AT+SPWM=0,0,0\r\n");
  }

  delay(500);
  SerialPrintLine("RVOL: " + String(rVol) + " CVOL:" + String(cVol), true); // Debug

  // INIT: Checking SIM card
  if (0 != gprs.init()) {
      SerialPrintLine("Not checked out to SIM card...", true); // Debug
      delay(1000);
      WriteText("Booting . . .", "Init failed: 1", 2000);
      SerialPrintLine("INIT FAIL: 1", true); // Debug
      while (1);
  }
  
  // INIT: Searching for BME280 sensor at I2C address 0x76
  SerialPrintLine("BME280 test", true); // Debug
  WriteText("Booting . . .", "BME280 TEST ...", 500);
  if (!bme.begin(0x76)) {
    SerialPrintLine("Could not find a valid BME280 sensor, check wiring!", true); // Debug
    WriteText("Booting . . .", "Init failed: 2", 2000);
    SerialPrintLine("INIT FAIL: 2", true); // Debug
    while (1);
  }
  SerialPrint("Temperature = "); // Debug
  SerialPrint(String(bme.readTemperature())); // Debug
  SerialPrintLine("*C"); // Debug

  // INIT: Sending startup AT commands
  SerialPrintLine("Sending startup commands", true); // Debug
  WriteText("Sending startup", "AT commands", 500);
  CMD("AT+CHFA=1"); // Set audio channel
  CMD("AT+CLIP=1"); // Enable CLIP notifications

  CMD("AT+CMGF=1"); // Set SMS message format to text mode
  CMD("AT+CNMI=1,2,0,0,0"); // Set SMS receive format
  CMD("AT+CBUZZERRING=0");  // Set incoming call ring NOT to buzzer sound
  CMD("AT+CEXTERNTONE=0");  // Enable microphone
  CMD("AT+CRSL=" + String(rVol));  // Set ringer volume level
  CMD("AT+CLVL=" + String(cVol));  // Set loud speaker volume level

  CMD("AT+CLTS=1"); // Enable get local time
  CMD("AT+CCLK?");  // Date and time
  CMD("AT+CSQ"); // Signal strength

  WriteText("Reading contacts", "", 1000);
  SerialPrintLine("Reading saved contacts", true); // Debug

  // INIT: Read contact list length
  int amount = preferences.getInt("AMOUNT", 0);
  NUMBER_AMOUNT = amount;
  SerialPrintLine("Found: " + String(amount), true); // Debug
  WriteText("Reading contacts", String(amount) + " found", 1000);

  // INIT: Read contact list
  for (int i = 0; i < amount; i++){
    String key = "NAME" + String(i);
    //SerialPrint(key + " "); // Debug
    String d = preferences.getString(key.c_str(), "N/A");
    //SerialPrint(d); // Debug
    names[i] = d;
    
    key = "NUMBER" + String(i);
    //SerialPrint(" " + key + " "); // Debug
    d = preferences.getString(key.c_str(), "N/A");
    //SerialPrintLine(d); // Debug
    numbers[i] = d;
  }
  
  // INIT: Sort contact list
  SerialPrint("SORTING CONTACT LIST . . . "); // Debug
  SortContactList(0, NUMBER_AMOUNT - 1);
  SerialPrintLine("DONE"); // Debug
  /*for (int i = 0; i < NUMBER_AMOUNT; i++) { // Debug
    SerialPrintLine(names[i] + " " + numbers[i]);    
  }*/
  
  // INIT: Init success
  //Serial.println("Init success"); // Debug
  SerialPrintLine("Init success", true);
  WriteText("Init success", "Soyuz V1.1", 2000);
}

#pragma region GlobalVariables
// Menu
int MAX_MENU_COUNT = 8;
bool msg = false;
int menuCount = 0;
int lastMenuCount = -1;
int numberCount = 0;
bool isScrollingNumber = false;
bool confirmDelete = false;
bool isCalling = false;
bool incoming = false;
bool setVolR = false;
bool setVolC = false;
int sec = -1;
int lastSec = -2;

// Readed data
float temp = 0;
float lastTemp = -1;
int mV = 0;
int signalStrength = 0;

// Dialing
bool isDialing = false;
String toDial = "";
char chars[] = "0123456789#";
char chars2[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ #";
int dCharIndex = 0;

// Contact save
int contactSave = 0;
String contactName = "";
String contactNumber = "";

bool isShowing = true;

// Messages
String messages[] = {};
int messagesLength = 0;

// Date and time
int Y = 0;
int M = 0;
int D = 0;
int h = 0;
int m = 0;
int s = 0;

int lastm = -1;
int batState = 0;
int lastBatState = -1;
int signalState = 0;
int lastSignalState = -1;
#pragma endregion GlobalVariables

void loop() {
  sec = millis() / 1000;

  // Update main screen data
  if (sec % 10 == 0 && menuCount == 0 && !incoming && !isCalling && sec != lastSec && contactSave == 0) {
    ShowBattery();
  }

  // Get temperature from sensor
  if (sec % 10 == 0) {
    temp = bme.readTemperature();
  }

  // Update time
  if (sec != lastSec) {
    lastSec = sec;
    UpdateTime();
  }
  
  // Hardware Serial event handling
  if (hwSerial.available()) {
    String s = hwSerial.readStringUntil('\n');
    if (s.indexOf("CMGR") > 0) {  //  READ SMS
      msg = true;
      SerialPrintLine(s);
    } else if (msg) { // READ SMS HEX
      msg = false;
      SerialPrintLine(hex(s));
    } else if (s.indexOf("CLIP") > 0){  //  INCOMING CALL
      SerialPrintLine("CALLING NAME8CHR");
      String caller = s.substring(8);
      caller = caller.substring(0, caller.indexOf('"'));
      if (caller.length() > 3){
        SerialPrintLine(caller);
        int index = getArrayIndex(numbers, caller);
        if (index >= 0){
          SerialPrintLine(names[index]);
          WriteText("Incoming call", names[index]);
        } else {
          SerialPrintLine("UNKNOWN");
          WriteText("Incoming call", caller);
        }
        incoming = true;
        isShowing = true;
        lcd.backlight();
      } 
      SerialPrintLine(s);
    } else if (s.indexOf("CCLK") > 0){ // Reading time - Data format: +CCLK: “18/06/21,12:00:21+22” DD/MM/YY,HH:MM:SS 8-10 11-13 14-16 17-19 20-22 23-25
      if (s.length() > 15){
        Y = s.substring(8, 10).toInt();
        M = s.substring(11, 13).toInt();
        D = s.substring(14, 16).toInt();
        h = s.substring(17, 19).toInt();
        m = s.substring(20, 22).toInt();
        s = s.substring(23, 25).toInt();
        SerialPrintLine(s);
      }
    } /*else if (s.indexOf("CBC:") > 0){ // Using other method to read battery voltage
      if (s.length() > 10) {
        mV = s.substring(11, 15).toInt(); //4030 max 3700 min
        //mV = mV - 3700;
        //mV = mV * 100 / 330;
        SerialPrintLine(s);
      }
    } */else if (s.indexOf("CSQ:") > 0){ // Signal strength
      SerialPrintLine(s);
      s.remove(0, 6);
      s.remove(s.indexOf(","));
      signalStrength = s.toInt();
      SerialPrintLine("Signal strength: " + String(signalStrength));
    } else if (s.indexOf("CMT") > 0) { // Trying out: listening to incoming SMS messages
      while (hwSerial.available()) {
        SerialPrintLine(hwSerial.readStringUntil('\n'));
      }
    } else {  //  DEFAULT
      SerialPrintLine(s);
    }
    SerialPrintLine("==========");
  }

  // Serial event handling
  if (Serial.available()) { // Debug
    String r = Serial.readString();
    if (isCMD(r)){
      SerialBT.println(r.c_str());
      exeCMD(r);
    } else if (r.indexOf("AT") >= 0) {
      hwSerial.write(r.c_str());
      SerialBT.println(r.c_str());
    }
  }

  // Bluetooth Serial event handling
  if (SerialBT.available()) { // Debug
    String r = SerialBT.readString();
    if (isCMD(r)){
      Serial.println(r);
      exeCMD(r);
    } else if (r.indexOf("AT") >= 0) {
      hwSerial.write(r.c_str());
      Serial.println(r);
    }
  }

  // Check input values
  sideButton.check();
  rotButton.check();
  byte pos = rotEnc.rotate();
  //button.check(); NOT IN USE

  if (isScrollingNumber) { // Scrolling in contact list
    if (pos == 1){
      numberCount--;
      if (numberCount < 0) {
        numberCount = NUMBER_AMOUNT - 1;
      }
      WriteText(names[numberCount], numbers[numberCount]);
    }
    if (pos == 2) {
      numberCount++;
      if (numberCount >= NUMBER_AMOUNT) {
        numberCount = 0;
      }
      WriteText(names[numberCount], numbers[numberCount]);
    }
  } else if (isShowing && !isDialing && contactSave == 0 && !setVolC && !setVolR) { // Scrolling in menu
    if (pos == 1) {
      lastMenuCount = menuCount;
      menuCount--;
      if (menuCount < 0) {
        menuCount = MAX_MENU_COUNT;
      }
      SerialPrintLine("Menu count: " + String(menuCount));
    }
    if (pos == 2) {
      lastMenuCount = menuCount;
      menuCount++;
      if (menuCount > MAX_MENU_COUNT) {
        menuCount = 0;
      }
      SerialPrintLine("Menu count: " + String(menuCount));
    }
    if (pos != 0) { // On menu count change
      switch (menuCount){
        case 0:
          ShowBattery();
          break;
        case 1:
          WriteText(String("Saved numbers"), String(NUMBER_AMOUNT) + " contacts");
          break;
        case 2:
          WriteText(String("Messages:"), String(messagesLength) + " unread");
          break;
        case 3:
          WriteText(String("Restart"));
          break;
        case 4:
          WriteText(String("Dial number"));
          break;
        case 5:
          WriteText(String("Save contact"));
          break;
        case 6:
          WriteText("Delete contact");
          break;
        case 7:
          WriteText("In call volume: ", String(inCallVol));
          break;
        case 8:
          WriteText("Ringer volume: ", String(ringerVol));
          break;
      }
    }
  } else if (isDialing) { // Dialing
    if (pos == 1) {
      dCharIndex--;
      if (dCharIndex < 0) {
        dCharIndex = 10;
      }
    } else if (pos == 2) {
      dCharIndex++;
      if (dCharIndex > 10) {
        dCharIndex = 0;
      }
    }
    if (pos != 0) {
      WriteText("Number to call:", toDial + chars[dCharIndex]);
    }
  } else if (contactSave == 1) { // Typing new contact name
    if (pos == 1) {
      dCharIndex--;
      if (dCharIndex < 0) {
        dCharIndex = 27;
      }
    } else if (pos == 2) {
      dCharIndex++;
      if (dCharIndex > 27) {
        dCharIndex = 0;
      }
    }
    if (pos != 0) {
      WriteText("Contact name:", contactName + chars2[dCharIndex]);
    }
  } else if (contactSave == 2) { // Typing new contact number
    if (pos == 1) {
      dCharIndex--;
      if (dCharIndex < 0) {
        dCharIndex = 10;
      }
    } else if (pos == 2) {
      dCharIndex++;
      if (dCharIndex > 10) {
        dCharIndex = 0;
      }
    }
    if (pos != 0) {
      WriteText("Contact number:", contactNumber + chars[dCharIndex]);
    }
  } else if (setVolC) { // Set in call volume
    if (pos == 1){
      inCallVol--;
      if (inCallVol < 0) {
        inCallVol = 0;
      }
      WriteText("In call volume: ", "> " + String(inCallVol));
    }
    if (pos == 2) {
      inCallVol++;
      if (inCallVol > 100) {
        inCallVol = 100;
      }
      WriteText("In call volume: ", "> " + String(inCallVol));
    }
  } else if (setVolR) { // Set ringer volume
    if (pos == 1){
      ringerVol--;
      if (ringerVol < 0) {
        ringerVol = 0;
      }
      WriteText("Ringer volume: ", "> " + String(ringerVol));
    }
    if (pos == 2) {
      ringerVol++;
      if (ringerVol > 100) {
        ringerVol = 100;
      }
      WriteText("Ringer volume: ", "> " + String(ringerVol));
    }
  }
}
bool showingMsg = false;

void handleEvent(AceButton* btn, uint8_t eventType, uint8_t buttonState) {
  // Debug
  /*SerialPrint(F("handleEvent(): eventType: "));
  SerialPrint(String(eventType));
  SerialPrint(F("; buttonState: "));
  SerialPrintLine(String(buttonState));*/

  switch (eventType) {
    case AceButton::kEventPressed:
      SerialPrintLine("Button pressed");
      switch (btn->getPin()) {
        case SIDEBTN_PIN:
          SerialPrintLine("SIDEBTN pressed"); // On side button press
          if (confirmDelete) {
            WriteText("Contact deleted", names[numberCount], 2000);
            RemoveContact(numberCount);
            confirmDelete = false;
            menuCount = 0;
            numberCount = 0;
            lastMenuCount = -1;
            ShowBattery();
            return;
          }
          if (contactSave == 1) {
            contactSave = 2;
            dCharIndex = 0;
            return;
          } else if (contactSave == 2) {
            dCharIndex = 0;
            //CMD("AT+CPBW=,\"+" + String(contactNumber) + "\",145,\"" + contactName + "\"");
            AddToContactList(contactName, contactNumber);
            contactSave = 0;
            menuCount = 0;
            contactName = "";
            contactNumber = "";
            return;
          }
          if (isDialing) {
            CMD("ATD+" + toDial + ";");
            isCalling = true;
            isDialing = false;
            WriteText("Calling:", toDial);
            return;
          }
          if (isScrollingNumber && menuCount != 6) {
            isScrollingNumber = false;
            CMD("ATD" + numbers[numberCount] + ";");
            WriteText("Calling:", names[numberCount]);
            isCalling = true;
            return;
          }
          if (isScrollingNumber && menuCount == 6) {
            isScrollingNumber = false;
            WriteText("Delete contact?", names[numberCount]);
            confirmDelete = true;
            return;
          }
          if (incoming) {
            incoming = false;
            CMD("ATA");
            isCalling = true;
            return;
          } else if (isCalling) {
            CMD("ATH");
            incoming = false;
            isCalling = false;
            WriteText("Call ended", "", 1000);
            ClearDisplay();
            ShowBattery();
            menuCount = 0;
            isScrollingNumber = false;
            numberCount = 0;
            return;
          }
          if (setVolC) {
            CMD("AT+CLVL=" + String(inCallVol));  // Set loud speaker volume level
            preferences.putInt("CVOL", inCallVol);
            WriteText("In call volume: ", String(inCallVol));
            setVolC = false;
            return;
          }
          if (setVolR) {
            CMD("AT+CRSL=" + String(ringerVol));  // Set ringer volume level
            preferences.putInt("RVOL", ringerVol);
            WriteText("Ringer volume: ", String(ringerVol));
            setVolR = false;
            return;
          }
          if (menuCount == 0 && contactSave == 0) { // ============================================================================
            if (isShowing) {
              isShowing = false;
              lcd.noBacklight();
            } else {
              isShowing = true;
              lcd.backlight();
            }
          } else if (menuCount == 1) { // ============================================================================
            SerialPrintLine("Scroll in list of numbers");
            if (isCalling) {
              CMD("ATH");
              isCalling = false;
              incoming = false;
              ClearDisplay();
              return;
            }
            WriteText(names[numberCount], numbers[numberCount]);
            isScrollingNumber = true;
          } else if (menuCount == 2) { // ============================================================================
            if (messagesLength > 0 && !showingMsg) {
              WriteText(messages[0], messages[1]);
              DeleteLastMessage();
              showingMsg = true;
            } else if (showingMsg) {
              showingMsg = false;
              WriteText("Messages:", messagesLength + " unread");
            }
          } else if (menuCount == 3) { // ============================================================================
            SerialPrintLine("Rebooting . . .");
            WriteText("Restarting . . .", "");
            delay(2000);
            lcd.noBacklight();
            lcd.clear();
            ESP.restart();
          } else if (menuCount == 4) { // ============================================================================
            isDialing = true;
            WriteText("Number to call:");
          } else if (menuCount == 5) { // ============================================================================
            if (NUMBER_AMOUNT == 250){
              WriteText("Contactlist full", "", 1500);
              return;
            }
            contactSave = 1;
            WriteText("Contact name:");
          } else if (menuCount == 6) { // ============================================================================
            isScrollingNumber = true;
            WriteText(names[numberCount], numbers[numberCount]);
          } else if (menuCount == 7) { // ============================================================================
            setVolC = true;
            WriteText("In call volume: ", "> " + String(inCallVol));
          } else if (menuCount == 8) { // ============================================================================
            setVolR = true;
            WriteText("Ringer volume: ", "> " + String(ringerVol));
          }
          break;
        case ROTBTN_PIN:
          SerialPrintLine("ROTBTN pressed"); // On rotary button press
          if (!isShowing) return;
          if (confirmDelete) {
            WriteText("Contact delete", "aborted", 2000);
            confirmDelete = false;
            menuCount = 0;
            numberCount = 0;
            lastMenuCount = -1;
            ShowBattery();
            return;
          }
          if (isDialing) {
            if (chars[dCharIndex] != '#') {
              toDial += chars[dCharIndex];
              WriteText("Number to call:", toDial + chars[dCharIndex]);
            } else {
              menuCount = 0;
              isDialing = false;
              dCharIndex = 0;
              toDial = "";
            }
            SerialPrintLine(toDial + "(" + chars[dCharIndex] + ")");
            return;
          }
          if (contactSave == 1) {
            if (chars2[dCharIndex] != '#') {
              contactName += chars2[dCharIndex];
              WriteText("Contact name:", contactName);
              //WriteLine1("Contact name:", true);
              //WriteLine2(contactName, true);
            } else {
              menuCount = 0;
              contactSave = 0;
              dCharIndex = 0;
              contactName = "";
            }
            return;
          } else if (contactSave == 2) {
            if (chars[dCharIndex] != '#') {
              contactNumber += chars[dCharIndex];
              WriteText("Contact number:", contactNumber);
              //WriteLine1("Contact number:", true);
              //WriteLine2(contactNumber, true);
            } else {
              menuCount = 0;
              contactSave = 0;
              dCharIndex = 0;
              contactNumber = "";
            }
            return;
          }
          if (incoming) {
            CMD("ATH");
            incoming = false;
            isCalling = false;
            return;
          }
          ClearDisplay();
          ShowBattery();
          menuCount = 0;
          numberCount = 0;
          isScrollingNumber = false;
          break;
      }
      break;
    case AceButton::kEventReleased: // No need for this
      SerialPrintLine("Button released");
      break;
    case AceButton::kEventDoubleClicked: // No need for this
      SerialPrintLine("Button double clicked");
      break;
  }
}

bool isCMD(String s) {
  if (s.indexOf("DEBUG=ON") == 0 || s.indexOf("DEBUG=OFF") == 0 || s.indexOf("RE") == 0 ||
      s.indexOf("RESTART") == 0 || s.indexOf("DEBUGBAT=ON") == 0 || s.indexOf("DEBUGBAT=OFF") == 0) {
    Serial.println("OK");
    SerialBT.println("OK");
    return true;
  }
  Serial.println("NOT DEV CMD - REDIRECT TO SIM800");
  Serial.println("==========");
  SerialBT.println("NOT DEV CMD - REDIRECT TO SIM800");
  SerialBT.println("==========");
  return false;
}

void exeCMD(String s) {
  if (s.indexOf("RE") >= 0){
    Serial.println("RE: OK");
    SerialBT.println("RE: OK");
  } else if (s.indexOf("DEBUG=ON") >= 0) {
    debug = true;
    preferences.putInt("DEBUG", 1);
    Serial.println("RE: DEBUG=ON");
    SerialBT.println("RE: DEBUG=ON");
  } else if (s.indexOf("DEBUG=OFF") >= 0) {
    debug = false;
    preferences.putInt("DEBUG", 0);
    Serial.println("RE: DEBUG=OFF");
    SerialBT.println("RE: DEBUG=OFF");
  } else if (s.indexOf("RESTART") >= 0) {
    Serial.println("RE: RESTART . . .");
    SerialBT.println("RE: RESTART . . . (Connection will be lost)");
    WriteText("Restarting . . .", "");
    delay(2000);
    lcd.noBacklight();
    lcd.clear();
    ESP.restart();
  } else if (s.indexOf("DEBUGBAT=ON") >= 0) {
    debugBattery = true;
    preferences.putInt("DEBUGBAT", 1);
    Serial.println("RE: DEBUGBAT=ON");
    SerialBT.println("RE: DEBUGBAT=ON");
  } else if (s.indexOf("DEBUGBAT=OFF") >= 0) {
    debug = false;
    preferences.putInt("DEBUGBAT", 0);
    Serial.println("RE: DEBUGBAT=OFF");
    SerialBT.println("RE: DEBUGBAT=OFF");
  }
  Serial.println("==========");
  SerialBT.println("==========");
}

#pragma region Custom_LCD_Chars
// Write custom battery character based on voltage reads
void WriteBatteryChar(int x, int y, int batteryPercent, bool condition) {
  byte b[] = { 0x0A, 0x15, 0x0A, 0x15, 0x0A, 0x15, 0x0A, 0x15};
  if (batteryPercent >= 90){
    byte c[] = { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    batState = 7;
  }
  else if (batteryPercent >= 75){
    byte c[] = { 0x0E, 0x19, 0x1D, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    batState = 6;
  }
  else if (batteryPercent >= 60){
    byte c[] = { 0x0E, 0x11, 0x19, 0x1D, 0x1F, 0x1F, 0x1F, 0x1F };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    batState = 5;
  }
  else if (batteryPercent >= 45){
    byte c[] = { 0x0E, 0x11, 0x11, 0x19, 0x1D, 0x1F, 0x1F, 0x1F };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    batState = 4;
  }
  else if (batteryPercent >= 30){
    byte c[] = { 0x0E, 0x11, 0x11, 0x11, 0x19, 0x1D, 0x1F, 0x1F };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    batState = 3;
  }
  else if (batteryPercent >= 5){
    byte c[] = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x19, 0x1D, 0x1F };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    batState = 2;
  }
  else if (batteryPercent >= 0){
    byte c[] = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x19, 0x1F };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    batState = 1;
  }
  if (condition){
    lcd.createChar(0, b);
    //lcd.home();
    lcd.setCursor(x, y);
    lcd.write(0);
    lastBatState = batState;
  }
}

// Write custom signal strength character based on signal strength
void WriteSignalChar(int x, int y, int signalValue, bool condition) {
  byte b[] = { 0x0A, 0x15, 0x0A, 0x15, 0x0A, 0x15, 0x0A, 0x15};
  if (signalValue >= 20 && signalValue != 99){
    byte c[] = { 0x01, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0F, 0x1F };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    signalState = 4;
  }
  else if (signalValue < 2 || signalValue == 99){
    byte c[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    signalState = 0;
  }
  else if (signalValue >= 2 && signalValue < 10){
    byte c[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x18 };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    signalState = 1;
  }
  else if (signalValue >= 10 && signalValue < 15){
    byte c[] = { 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x0C, 0x1C };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    signalState = 2;
  }
  else if (signalValue >= 15 && signalValue < 20){
    byte c[] = { 0x00, 0x00, 0x02, 0x02, 0x06, 0x06, 0x0E, 0x1E };
    for (int i = 0; i < 8; i++) b[i] = c[i];
    signalState = 3;
  }
  if (condition){
    lcd.createChar(1, b);
    //lcd.home();
    lcd.setCursor(x, y);
    lcd.write(1);
    lastSignalState = signalState;
  }
}

// Write custom character if got any messages
void WriteMessageChar(int x, int y, bool condition) {
  byte b[] = { 0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0x00, 0x0A, 0x00};
  lcd.createChar(2, b);
  //lcd.home();
  lcd.setCursor(x, y);
  if (condition) lcd.write(2);
  else lcd.print(" ");
  lastSignalState = signalState;
}
#pragma endregion Custom_LCD_Chars

#pragma region Contactlist
// Add new contact to list
void AddToContactList(String cName, String cNum) {
  SerialPrintLine("Adding to contacts: " + cName + " +" + cNum);
  WriteText("New contact:", "", 1500);
  WriteText(FormatContact(cName), cNum, 1500);
  
  numbers[NUMBER_AMOUNT] = "+" + cNum;
  names[NUMBER_AMOUNT] = FormatContact(cName);
  NUMBER_AMOUNT++;

  SortContactList(0, NUMBER_AMOUNT - 1);
  SaveContactList();
  WriteText("Contact saved", "", 1500);
}

// Sort contact list by name
void SortContactList(int low, int high) {
  if (low < high)
  {
    int pi = partition(low, high);

    SortContactList(low, pi - 1);  // Before pi
    SortContactList(pi + 1, high); // After pi
  }
}

// Save contact list to flash memory
void SaveContactList() {
  preferences.putInt("AMOUNT", NUMBER_AMOUNT);
  for (int i = 0; i < NUMBER_AMOUNT; i++){
    String key = "NAME" + String(i);
    preferences.putString(key.c_str(), names[i]);
    key = "NUMBER" + String(i);
    preferences.putString(key.c_str(), numbers[i]);
  }
  SerialPrintLine("Contact list saved " + String(NUMBER_AMOUNT));
}

// Remove contact from list (and from flash) and save
void RemoveContact(int i) {
  SerialPrintLine("REMOVING CONTACT " + String(i));
  SerialPrintLine(names[i] + " " + numbers[i]);
  for (int j = i; j < NUMBER_AMOUNT - 1; j++) {
    names[j] = names[j + 1];
    numbers[j] = numbers[j + 1];
  }
  names[NUMBER_AMOUNT - 1] = "";
  numbers[NUMBER_AMOUNT - 1] = "";
  preferences.remove(("NAME" + String(NUMBER_AMOUNT - 1)).c_str());
  preferences.remove(("NUMBER" + String(NUMBER_AMOUNT - 1)).c_str());
  NUMBER_AMOUNT--;
  SaveContactList();
}

// Fromatting contact name (FIRSTNAME LASTNAME ==> Firstname Lastname)
String FormatContact(String inU) {
  String o = "";
  String inL = inU;
  inL.toLowerCase();
  for (int i = 0; i < inU.length(); i++) {
    if (i > 0){
      if (inU[i] != ' ' && inU[i - 1] == ' '){
         o += inU[i];
      }
      else {
        o += inL[i];
      }
    }
    else {
      o += inU[i];
    }
  }
  SerialPrintLine("OLD: " + inU);
  SerialPrintLine("NEW: " + o);
  return o;
}

#pragma region Sort
int partition(int low, int high) {
  // pivot (Element to be placed at right position)
  String pivot = names[high];  

  int i = (low - 1);  // Index of smaller element and indicates the 
                      // right position of pivot found so far

  for (int j = low; j <= high - 1; j++)
  {
    // If current element is smaller than the pivot
    if (names[j] < pivot)
    {
      i++;    // increment index of smaller element
      swap(i, j);
    }
  }
  swap(i + 1, high);
  return (i + 1);
}

void swap(int a, int b) {
  String temp = names[a];
  names[a] = names[b];
  names[b] = temp;

  temp = numbers[a];
  numbers[a] = numbers[b];
  numbers[b] = temp;
}
#pragma endregion Sort
#pragma endregion Contactlist

#pragma region LCD_Functions
void WriteLine1(String line, bool c) {
  lcd.setCursor(0, 0);
  if (c) lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print(line);
}

void WriteLine2(String line, bool c) {
  lcd.setCursor(0, 1);
  if (c) lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(line);
}

void WriteText(String line1) {
  WriteText(line1, true);
}

void WriteText(String line1, bool c) {
  if (c){
    WriteText(line1, String(""));
  } else {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.print(line1);
  }
}

void WriteText(String line1, String line2) {// oszlop, sor
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void WriteText(String line1, String line2, int d) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
  delay(d);
}

void ClearDisplay() {
  lcd.clear();
}
#pragma endregion LCD_Functions

// Show date, time, temperature, battery percentage, signal strength
void ShowBattery() {
  int b = getBatteryValue();
  String bat = String(b);
  if (b < 100 && b > 9) {
    bat = " " + bat;
  } else if (b < 10) {
    bat = "  " + bat;
  }
  String tMonth = "";
  if (M < 10) {
    tMonth = "0";
    tMonth += String(M);
  } else tMonth = String(M);
  String tDay = "";
  if (D < 10) {
    tDay = "0";
    tDay += String(D);
  } else tDay = String(D);
  String tHour = "";
  if (h < 10) {
    tHour = "0";
    tHour += String(h);
  } else tHour = String(h);
  String tMinute = "";
  if (m < 10) {
    tMinute = "0";
    tMinute += String(m);
  } else tMinute = String(m);
  if (lastMenuCount != menuCount) {
    WriteText("20" + String(Y) + "." + tMonth + "." + tDay + "." + /*bat + "%",*/ tHour + ":" + tMinute,/* + " " + /*(t < 10 ? " " + String(t) : String(t))*/String(temp) + "C " + String(/*mV*/(int)b) + "%");
    WriteBatteryChar(14, 1, (int)b, true);
    WriteSignalChar(15, 1, signalStrength, true);
    WriteMessageChar(13, 1, messagesLength > 0);
    lastMenuCount = menuCount;
  } else {
    if (lastm != m) {
      WriteLine1("20" + String(Y) + "." + tMonth + "." + tDay + "." + /*bat + "%",*/ tHour + ":" + tMinute, false);
      //WriteText("20" + String(Y) + "." + tMonth + "." + tDay + "." + /*bat + "%",*/ tHour + ":" + tMinute,/* + " " + /*(t < 10 ? " " + String(t) : String(t))*/String(temp) + "C " + String(/*mV*/(int)b) + "%");
      lastm = m;
    }
    if (lastTemp != temp) {
      WriteLine2(String(temp) + "C " + String(/*mV*/(int)b) + "% ", lastMenuCount != menuCount);
      lastTemp = temp;
    }
    //WriteBatteryChar(14, 1, (int)b, lastBatState != batState);
    //WriteSignalChar(15, 1, signalStrength, lastSignalState != signalState);
    WriteBatteryChar(14, 1, (int)b, true);
    WriteSignalChar(15, 1, signalStrength, true);
    WriteMessageChar(13, 1, messagesLength > 0);
  }
}

// Get index of value in array
int getArrayIndex(String arr[], String val) {
  for (int i = 0; i < arr->length(); i++) {
    if (arr[i] == val) {
      return i;
    }
  }
  return -1;
}

// Request date, time, signal strength
void UpdateTime() {
  s++;
  if (s % 10 == 0 && signalStrength == 0) {
    CMD("AT+CSQ");
  }
  if (s == 60) {
    s = 0;
    lastm = m;
    m++;
    CMD("AT+CSQ");
  }
  if (m == 60) {
    m = 0;
    h++;
  }
  if (h == 24) {
    h = 0;
    CMD("AT+CCLK?");
  }
}

void AddMessage(String lineA, String lineB) {
  messagesLength += 2;
  int msgLength = messagesLength - 2;
  String tmp[msgLength];
  for (int i = 0; i < msgLength; i++) {
    tmp[i] = messages[i];
  }
  String updated[msgLength + 2];
  for (int i = 2; i < msgLength + 2; i++) {
    updated[i] = tmp[i - 2];
  }
  updated[0] = lineA;
  updated[1] = lineB;
  for (int i = 0; i < msgLength + 2; i++) {
    messages[i] = updated[i];
  }
}

void DeleteLastMessage() {
  messagesLength -= 2;
  int msgLength = messagesLength + 2;
  String tmp[msgLength];
  SerialPrintLine("A1");
  for (int i = 0; i < msgLength; i++) {
    tmp[i] = messages[i];
  }
  SerialPrintLine("A2");
  String updated[msgLength - 2];
  SerialPrintLine("A3");
  for (int i = 0; i < msgLength - 2; i++) {
    updated[i] = tmp[i + 2];
  }
  SerialPrintLine("A4");
  for (int i = 0; i < msgLength - 2; i++) {
    messages[i] = updated[i];
  }
  SerialPrintLine("A5");
}

#pragma region AT_Functions
void CMD(String c) {
  hwSerial.print(c + "\r\n");
  delay(500);
}

void CMD_Print(String c) {
  hwSerial.print(c);
}

void CMD_End() {
  hwSerial.print("\r\n");
  delay(500);
}

void CMD_Write(int i) {
  hwSerial.write(i);
  delay(500);
  hwSerial.print("\r\n");
  delay(500);
}
#pragma endregion AT_Functions

#pragma region Battery_Functions
// This part is a real mess

int batReads[] { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
// Avarage last battery reads
int AvgBattValues() {
  int count = 0;
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    if (batReads[i] != -1){
      count++;
      sum += batReads[i];
    }
  }
  return sum / count;
}

// Push new value to array
void NewBatteryRead(int bRead) {
  for (int i = 0; i < 9; i++){
    batReads[i] = batReads[i + 1];
  }
  batReads[9] = bRead;
}

// Debug
void PrintBatReads() {
  for (int i = 9; i > 0; i--) {
    SerialPrint(String(batReads[i]) + ", ");
  }
  SerialPrintLine(String(batReads[0]));
  SerialPrintLine("==========");
}

// Read battery percentage
int getBatteryValue() {//1575
  float c = analogRead(BATT_V);
  if (debugBattery) SerialPrint("ANALOG: " + String(c));
  c = map(c, 0, 4095, 0, 3440);
  if (debugBattery) SerialPrint(" MAP: " + String(c));
  c = ((c + 40 - 1850) * 10) / (float)25;
  if (debugBattery) SerialPrint(" FINAL: " + String(c));
  NewBatteryRead(c);
  c = AvgBattValues();
  c += 10;          // OFFSET BY +10%
  if (debugBattery) SerialPrintLine(" AVG: " + String(c));
  if (debugBattery) PrintBatReads();
  if (c > 100) c = 100;
  if (c < 0) c = 0;
  return c;
}
#pragma endregion Battery_Functions

// NOT IN USE YET
String hex(String s) {
  String out = "";
  for (int i = 0; i < s.length(); i+=2) {
    char a = s[i];
    char b = s[i + 1];
    char c[2];
    c[0] = a;
    c[1] = b;
    int fuck = strtol(c, NULL, 16);
    char d = fuck;
    out += d;
  }
  return out;
}

void SendSMS(String to, String msg) {
  delay(5000);
  SerialPrintLine("Send sms to " + to + " : " + msg);
  CMD("AT+CMGF=1");
  CMD("AT+CMGS=\"" + to + "\"");
  CMD(msg);
  CMD_Write(26);
}

void ReadLastSMS() {
  delay(1000);
  SerialPrintLine("Last sms");
  CMD("AT+CMGF=1");
  CMD("AT+CMGR=1");
  delay(2000);
  if (hwSerial.available()) {
    for (int i = 0; i < 2; i++) {
      String a = hwSerial.readStringUntil('\n');
    }
    String s = hwSerial.readStringUntil('\n');
    SerialPrintLine("LSMS: " + s);
    delay(100);
    s = hwSerial.readStringUntil('\n');
    SerialPrintLine("LSMS: " + s);
    delay(100);
    s = hwSerial.readStringUntil('\n');
    SerialPrintLine("LSMS: " + hex(s));
  }//Need to delete after read
}
