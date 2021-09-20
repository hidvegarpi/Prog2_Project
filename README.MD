Project Soyuz
=====================

This project started with a __simple__ idea, let's make a phone from an old radio. The radio, which from the design started, is from one of my old friend, who found it at a garbage disposal.

Hardware
--------

* [LILYGO TTGO T-CALL ESP32 SIM800L](https://www.aliexpress.com/item/33045221960.html "Order here")
* [1602 LCD Display](https://www.hestore.hu/prod_10036401.html "Order here")
* [I2C LCD Driver](https://www.hestore.hu/prod_10035516.html "Order here")
* [BME280 Temperature and humidity sensor](https://www.hestore.hu/prod_10039068.html "Order here")
* [Rotary encoder](https://www.hestore.hu/prod_10040813.html "Order here")
* [Push button](https://www.hestore.hu/prod_10028247.html "Order here")
* [2 x Rocker switch](https://www.hestore.hu/prod_10028128.html "Order here")
* [Slide switch](https://www.hestore.hu/prod_10042063.html "Order here")
* 4 x White led
* 4 x 470 ohm resistor
* 2 x 220 kohm resistor
* Microphone
* 8 ohm speaker
* Li-ion battery charger
* [DC-DC Boost converter](https://www.hestore.hu/prod_10042081.html "Order here")
* [5V Voltage regulator LM1084IT-5.0](https://www.hestore.hu/prod_10026480.html "Order here")
* 3 x 18650 battery
* [3 x 18650 battery case](https://www.hestore.hu/prod_10037304.html "Order here")
* 2 x Custom PCB, designs included here ([you can order them here](https://jlcpcb.com/ "JLC PCB"))

Needed for assembly
-------------------

* Programming
	* Arduino IDE
	* USB-C cable
* Mounting material
	* Wire
	* Solder wire
	* Insulating tape
	* Heatshrink
* Tools
	* Soldering iron
	* Wire cutter
	
The menu of the phone
---------------------

1. Main page (shows date, time, battery charge, temperature, signal strength)
1. Contact list (saved contacts readed from flash memory of ESP32)
1. Messages (you get a message, if you have a missed call)
1. Restart (reboot the phone)
1. Dial number (you can dial a number if you wan to call without saving it)
1. Save contact (save a phone number with a contact name)
1. Delete contact (no need to explain)

You can select the above menu items by pressing the sidebutton, the rotary encoder is used for scrolling between them.
If the sidebutton is pressed, while on the main page, the backlight will turn OFF/ON.
If the backlight is off, any input will be ignored except for one, the sidebutton is used to turn the backlight on again.

3D render of internal components
--------------------------------

![Early render](https://github.com/hidvegarpi/Prog2_Project/blob/main/early_render_of_internal_components_1.png)
*Early render*
