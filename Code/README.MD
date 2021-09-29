Code will be uploaded when working only with minor bugs
===
The actual program of the project
===

The program is written in the [Arduino IDE]("https://www.arduino.cc/en/software" "Download here").
A few libraries, and board managers need to be downloaded for the code to compile.

AT Commands example<sup>[1](#myfootnote1)</sup>
---

AT commands are sent through a I2C connection from the ESP32 to the SIM800 module.
We need to create this connection first.
In the example below, we define the global variables needed and start the serial connection.\
__NOTE:__ The following functions take relatively long time to execute, because it has to communicate with other hardware.

```C++
// Importing libraries
#include <sim800.h>
#include <gprs.h>

// Defining global variables
#define T_CALL

#define UART_TX                     27
#define UART_RX                     26
#define SIMCARD_RST                 5
#define SIMCARD_PWKEY               4
#define SIM800_POWER_ON             23

#define UART_BAUD_RATE              9600

HardwareSerial hwSerial(1);
GPRS gprs(hwSerial, SIMCARD_PWKEY, SIMCARD_RST, SIM800_POWER_ON);

void Setup() {
    Serial.begin(9600); // Serial for debug reasons
    hwSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX, UART_TX); // Start the serial interface
    CMD("AT");  // Sending the first AT command, if the 
}
```

We need some functions to send AT commands to the SIM800 module, because we are programmers and we do not want to write the same long code every time we send a command. In the example below, we create the functions needed to send commands easily.

```C++
void CMD(String c) { // Send a line through serial
  hwSerial.print(c + "\r\n");
  delay(500);
}

void CMD_Print(String c) { // Print to serial without line end
  hwSerial.print(c);
}

void CMD_End() { // End the line on serial
  hwSerial.print("\r\n");
  delay(500);
}

void CMD_Write(int i) { // Write a character (defined by an integer) to serial and end the line
  hwSerial.write(i);
  delay(500);
  hwSerial.print("\r\n");
  delay(500);
}
```

In the next example we use some of the above functions.
The example is capable of send an SMS message to a phone number.

__Important:__ the SIM800 module can only recognize international phone numbers, so for Hungary it looks something like the following (without the spaces): +36 20 123 4567

```C++
void SendSMS(String to, String msg) {
  Serial.println("Send sms to " + to + " : " + msg); // Print message to serial for debug reasons
  CMD("AT+CMGF=1");                                  // Seting SMS mode to text
  CMD("AT+CMGS=\"" + to + "\"");                     // Sending SMS message to "to"
  CMD(msg);                                          // Sending the actual message to serial
  CMD_Write(26);                                     // Ending the SMS with Ctrl+Z
}
```

I2C LCD example<sup>[2](#myfootnote2)</sup>
---

To tell the user what is actually happening there is a 16 character wide, 2 line LCD display with
an I2C LCD driver for easier wiring and control. In this example we will start an I2C connection
with the driver.\
__NOTE:__ The following functions take relatively long time to execute, because it has to communicate with other hardware.

```C++
// Importing libraries
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Defining global variables
#define I2C_SDA                     21
#define I2C_SCL                     22

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
    Wire.begin(I2C_SDA, I2C_SCL);
    lcd.begin();
}
```

We again, for easier use, will create some functions to control the LCD display.
We will need some way to write to the first and second line independently of one another.

```C++
void WriteLine1(String line, bool c) {    // Write text to first line
    lcd.setCursor(0, 0);                  // Set cursor position to first character of first line
    if (c) lcd.print("                "); // If c, then clear line before writing
    lcd.setCursor(0, 0);
    lcd.print(line);                      // Write the text
}

void WriteLine2(String line, bool c) {    // Write text to second line
    lcd.setCursor(0, 1);                  // Set cursor position to first character of second line
    if (c) lcd.print("                "); // If c, then clear line before writing
    lcd.setCursor(0, 1);
    lcd.print(line);                      // Write the text
}

void WriteText(String line1, String line2) {   // Write text to both lines
    lcd.clear();                               // Clear display
    lcd.setCursor(0, 0);                       // Set cursor to first line
    lcd.print(line1);                          // Write first line
    lcd.setCursor(0, 1);                       // Set cursor to second line
    lcd.print(line2);                          // Write second line
}
```

We can toggle the backlight of the LCD for power saving or visibility reasons with the following lines:

```C++
lcd.backlight();   // Toggle backlight on
lcd.noBacklight(); // Toggle backlight off
```

BME280 example<sup>[3](#myfootnote3)</sup>
---

For real time temperature data, we use a BME280 sensor which communicates over I2C (what a surprise).
As we already learned above, first we need to estabilish connection with the device.\
__NOTE:__ The following functions take relatively long time to execute, because it has to communicate with other hardware.

```C++
// Importing libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Defining global variables
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

void setup() {
    Serial.begin(9600); // Serial for debug reasons

    if (!bme.begin(0x76)) { // Check if there is a BME280 sensor at address 0x76
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    Serial.println("Found device"); // Found a BME280 sensor at address 0x76
}
```

Now we have connection, so we can get data from the sensor.

```C++
void WriteTemp() { // Write temperature to serial
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println("*C");
}

void WritePres() { // Write pressure to serial
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println("hPa");
}

void WriteAlt() { // Write aprox. altitude to serial
    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println("m");
}

void WriteHum() { // Write humidity to serial
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println("%");
}
```

Footnotes:
---
<a name="myfootnote1">1</a>: Source: various websites, the official SIM800 [documentation](https://www.elecrow.com/wiki/images/2/20/SIM800_Series_AT_Command_Manual_V1.09.pdf "PDF"), library included examples.\
<a name="myfootnote2">2</a>: Source: various [websites](https://create.arduino.cc/projecthub/akshayjoseph666/interface-i2c-16x2-lcd-with-arduino-uno-just-4-wires-273b24 "Good example"), library included examples.\
<a name="myfootnote3">3</a>: Source: various [websites](https://lastminuteengineers.com/bme280-arduino-tutorial/ "Good example"), library included examples.