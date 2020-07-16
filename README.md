<h1>Azure IoT Hub with Arduino and visualization with Power BI</h1>

In this project the microcontroller we are using has two chips integred. ATMega and ES8266.
So we have two differente sketches. 

1. Select the board ATMEGA2560.
2. PIN MODE:
        1-2-3-4 ON 
        Rest    OFF
        RX/TX   0
3. Upload the corrrespondet sketch ATMEGA2560.ino

4. Select the board LOLIN(WEMOS) DR1 & DR2 mini (instruccions below to install it).
5. PIN MODE:
        5-6-7   ON 
        Rest    OFF
        RX/TX   0
6. And upload the correspondent sketch Azure_IoT_ES8266.ino

7. Switch the pin modes to connect the two chips (ATMega and ES8266) 
   PIN MODE:
        1-2-3-4 ON 
        Rest    OFF
        RX/TX   3

8. And finally open the Power BI application to see the data streaming:



Instructions to install the ESP8266 boards:
<ul>
<li>Start Arduino and open Preferences window.</li>
<li>Enter http://arduino.esp8266.com/stable/package_esp8266com_index.json into Additional Board Manager URLs field. You can add multiple URLs, separating them with commas.</li>
<li>Open Boards Manager from Tools > Board menu and find esp8266 platform.</li>
<li>Select the version 2.5.2 from a drop-down box.</li>
<li>Click install button.</li>
<li>Don't forget to select your LOLIN(WEMOS) DR1 & DR2 mini board from Tools > Board menu after installation.</li>
</ul>

