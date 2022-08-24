#include "e220.h"

#include <SoftwareSerial.h>

e220Module::e220Module(uint8_t rx, uint8_t tx) {
    this->rxPin = rx;
    this->txPin = tx;
}

e220Module::e220Module(uint8_t rx, uint8_t tx, uint8_t aux) {
    this->rxPin = rx;
    this->txPin = tx;
    this->auxPin = aux;
}

e220Module::e220Module(uint8_t rx, uint8_t tx, uint8_t aux, uint8_t m0, uint8_t m1) {
    this->rxPin = rx;
    this->txPin = tx;
    this->auxPin = aux;
    this->m0Pin = m0;
    this->m1Pin = m1;
}

bool e220Module::begin(int baudRate) {
    // Set the pins we have configured.
    if (this->auxPin != -1) {
        pinMode(this->auxPin, auxPin == 16 ? INPUT : INPUT_PULLUP);
    }
    if (this->canSetModes()) {
        pinMode(this->m0Pin, OUTPUT);
        digitalWrite(this->m0Pin, HIGH);
        pinMode(this->m1Pin, OUTPUT);
        digitalWrite(this->m1Pin, HIGH);
        this->_mode = E220_MODE_PROGRAM;
    }

#ifdef E220_DEBUG
    Serial.print("AUX Pin ");
    Serial.print(this->auxPin);
    Serial.print(" -> ");
    Serial.println(digitalRead(this->auxPin));

    Serial.print("M0 Pin ");
    Serial.print(this->m0Pin);
    Serial.print(" -> ");
    Serial.println(digitalRead(this->m0Pin));

    Serial.print("M1 Pin ");
    Serial.print(this->m1Pin);
    Serial.print(" -> ");
    Serial.println(digitalRead(this->m1Pin));
#endif /* E220_DEBUG */

    SoftwareSerial *ss = new SoftwareSerial(this->txPin, this->rxPin);
    ss->setTimeout(100);
    (*ss).begin(baudRate);
    this->stream = ss;

    if (! this->setMode(E220_MODE_NORMAL)) {
        _lastError = E220_INVALID_PARAM;
        ss->end();
        this->stream = NULL;
        return false;
    }

    ModuleStatus res = this->readConfig();
    if (res != E220_SUCCESS || _channel == -1) {
        _lastError = res;
        ss->end();
        this->stream = NULL;
        return false;
    }
    return true;
}

static volatile bool worRecvd = false;

bool e220Module::available() {
    if (_worListener) {
        return worRecvd;
    }
    return this->stream->available() > 0;
}

ICACHE_RAM_ATTR void wakeUp() {
     worRecvd = true;
}

bool e220Module::setupWORListener(int period) {
    if (auxPin == -1) {
        Serial.println("MUST have the AUX pin attached for WOR receiver operations.");
        return E220_INVALID_PARAM;
    }
    if (! setWORPeriod(period)) {
        _lastError = E220_INVALID_PARAM;
        return false;
    }
    if (_dirty && !setModuleConfiguration(false))
        return false;
    Serial.println("Changing mode to WOR_RECEIVER...");
    if (! setMode(E220_MODE_WOR_RX))
        return false;
    Serial.println("Attaching interrupt listener...");
    attachInterrupt(auxPin, wakeUp, FALLING);
    _worListener = true;
    return true;
};

bool e220Module::stopWORListener() {
    if (auxPin == -1) {
        Serial.println("MUST have the AUX pin attached for WOR receiver operations.");
        return E220_INVALID_PARAM;
    }
    detachInterrupt(auxPin);
    if (! setMode(E220_MODE_NORMAL))
        return false;
    _worListener = false;
    return true;
};

bool e220Module::checkWOR() {
    return worRecvd;
}

void e220Module::resetWOR() {
    worRecvd = false;
}

bool e220Module::setupWORSender(int period) {
    if (auxPin == -1) {
        Serial.println("MUST have the AUX pin attached for WOR receiver operations.");
        return E220_INVALID_PARAM;
    }
    if (! setWORPeriod(period)) {
        _lastError = E220_INVALID_PARAM;
        return false;
    }
    // WOR Changes are only set temporarily when changed via this function.
    if (_dirty && !setModuleConfiguration(false))
        return false;
    Serial.println("Changing mode to WOR_TRANSMITTER...");
    if (! setMode(E220_MODE_WOR_TX))
        return false;

    return true;
}

