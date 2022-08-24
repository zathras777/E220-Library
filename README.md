# Ebyte e220 Module Linrary

## What

This small library is intended to simplify communicating with the Ebyte E220 LoRa modules.

It is inspired by https://github.com/xreef/EByte_LoRa_E220_Series_Library

This library takes a different approach to the configuration of the module and aims to simplify the sending and receiving process. 
It is aimed at ESP8266 devices as that is what I am using with it. Maybe it will prove useful to someone else.

## Example

Everyone loves a good example :-)

``` c++
#include "e220.h"

// Pin order is RX, TX, AUX, M0, M1
e220Module e220(D3, D4, D5, D7, D6);

void setup() {
    Serial.begin(74880);
    delay(500);

    while (! e220.begin(9600)) {
        Serial.println("Waiting for device to be available...");
        Serial.println(e220.getLastErrorDescription());
        delay(5000);
    }

    e220.resetConfiguration(true);

    if (! e220.isRSSI()) {
        e220.setRSSI(true);
    }

    if (e220.hasConfigurationChanged()) {
        Serial.println("Configuration has changed, writing to module now.");
        e220.setModuleConfiguration(true);
        Serial.println(e220.getLastErrorDescription());
    }
    Serial.println(e220.getLastErrorDescription());
    e220.printConfiguration();
}

void loop() {
    e220.sendMessage("Hello World!");
    delay(2000);

    if (e220.available()) {
        Serial.println("Received response!");

        ModuleMessage msg;
        if (e220.receiveMessages(&msg)) {
            Serial.println(msg.asString());
            if (e220.isRSSI()) {
                Serial.print("RSSI: ");
                Serial.print(msg.rssi);
                Serial.print(", channel noise estimated at ");
                Serial.print(msg.channelNoise());
                Serial.println(" dBm");
            }
        } else {
            Serial.print("Error getting messages. ");
            Serial.println(e220.getLastErrorDescription());
        }
        // Wait 28 seconds...
        delay(28000);
    }
    // Wait 2 seconds before trying again.
    delay(2000);
}
```

```
Configuration has changed, writing to module now.
1
OK
----------------------------------------
       Current Configuration
----------------------------------------
Address : 0x000C
Channel : 0x00 [ 0 ] @ 850.13MHz
Serial: Parity : 28
        Speed  : 9600bps
Air Data Rate  : 2400bps
Sub Packet Size: 200 bytes
---------------
Transmit: Power             : 22db
          WOR Period        : 2000ms
          Mode              : Transparent
          LBT Enabled?      No
          RSSI : Enabled?               Yes
               : Ambient Noise Enabled? No
----------------------------------------
```

## Wake On Receive (WOR)

### Receiver

Setting up the receiver is done via setupWORListener specifying the WOR period to be used. Sending and receiving can be done using the normal functions and the various mode changes will be performed automatically.
Additionally the available() function is WOR aware and should work as expected.

```c++
void setup() {
    ...
    e220.setupWORListener(2000);
    ...
}

void loop() {
    if (e220.available()) {
        Serial.println("WOR Received!");
        e220.resetWOR();
        ModuleMessage msg;
        if (e220.receiveMessages(&msg)) {
            Serial.println(msg.asString());
            ...
        }
        // Important - MUST be after all other module operations
        e220.resetWOR();
    }
    ...
}
```

### Transmitter

Setting up the WOR transmitter is done via the setupWORSender giving the WOR period to use. All usual sending and receiving functions can be used.

```c++
void setup() {
    ...
    e220.setupWORSender(2000);
    ...
}
```

## Debug

To have the additional debug output generated, enable E220_DEBUG or E220_DEBUG_2 in the e220.h file or via the command line definitions for your build. 

## LoRa Module

Testing has been done with the eByte E220-900T22D module. https://www.cdebyte.com/products/E220-900T22D

## Bugs

I'm sure there will be many, so pull requests are welcomed :-)

