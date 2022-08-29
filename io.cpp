#include "e220.h"

#define RETURN_AND_SET(x)      \
    if (x != E220_SUCCESS) {   \
        _lastError = x;        \
        return false;          \
    }

/**
 * @brief Delay while making sure that we don't overflow the size of an unsigned long.
 *
 * If millis() is about to overflow then worst case is we wait slightly longer than we
 * should have.
 * 
 * @param timeout milliseconds delay
 */
static void managedDelay(unsigned long timeout) {
	unsigned long t = millis();
	if (((unsigned long) (t + timeout)) == 0){
		t = 0;
	}
	while ((millis() - t) < timeout) 	{ 	}
}

ModuleStatus e220Module::setMode(ModuleMode n) {
    if (! this->canSetModes())
        return E220_MODE_ERROR;

    switch (n) {
        case E220_MODE_NORMAL:
            digitalWrite(this->m0Pin, LOW);
            digitalWrite(this->m1Pin, LOW);
            break;
        case E220_MODE_WOR_TX:
            digitalWrite(this->m0Pin, HIGH);
            digitalWrite(this->m1Pin, LOW);
            break;
        case E220_MODE_WOR_RX:
            digitalWrite(this->m0Pin, LOW);
            digitalWrite(this->m1Pin, HIGH);
            break;
        case E220_MODE_PROGRAM:
            digitalWrite(this->m0Pin, HIGH);
            digitalWrite(this->m1Pin, HIGH);
            break;
    }

    managedDelay(40);

	if (waitCompleteResponse(1000, 100) == E220_SUCCESS){
		_mode = n;
#ifdef E220_DEBUG
        Serial.print("Mode changed to ");
        Serial.println(_mode);
#endif /* E220_DEBUG */
        return E220_SUCCESS;
	}
    return E220_MODE_ERROR;
}

bool e220Module::getModuleConfiguration() {
    ModuleStatus res = readConfig();
    _lastError = res;
    return res == E220_SUCCESS;
}

ModuleStatus e220Module::readConfig() {
    ModuleMode prevMode = this->_mode;
    ModuleStatus res = setMode(E220_MODE_PROGRAM);
    if (res != E220_SUCCESS)
        return res;   

    res = this->sendCommand(CMD_CFG_READ, REG_CFG, 6);
    if (res != E220_SUCCESS) {
#ifdef E220_DEBUG
        Serial.print("readConfig: Failed to send command: ");
        Serial.println(res);
#endif /* E220_DEBUG */
        this->setMode(prevMode);
        return res;
    }

    uint8_t cfgData[9]; // 3 byte command and 6 byte configuration
    res = this->receiveBytes(&cfgData[0], 9);
    this->setMode(prevMode);

    if (res != E220_SUCCESS)
        return res;
    if (cfgData[0] != CMD_CFG_READ || cfgData[1] != REG_CFG || cfgData[2] != 6)
        return E220_INVALID_PARAM;

    parseConfigurationBytes(&cfgData[3]);
    this->_dirty = false;
    return res;
}

bool e220Module::setModuleConfiguration(bool save) {
    ModuleMode prevMode = this->_mode;
    ModuleStatus res = setMode(E220_MODE_PROGRAM);
    RETURN_AND_SET(res)

    uint8_t cfgData[9] = { save ? CMD_CFG_SAVE : CMD_CFG_TEMP, REG_CFG, 6};
    uint8_t *newCfg = buildConfigurationBytes();
    memcpy(&cfgData[3], newCfg, 6);
    free(newCfg);

    res = this->sendBytes(&cfgData[0], 9);
    if (res != E220_SUCCESS) {
        this->setMode(prevMode);
        _lastError = res;
        return false;
    }

    res = this->receiveBytes(&cfgData[0], 9);
    this->setMode(prevMode);

    RETURN_AND_SET(res)

    parseConfigurationBytes(&cfgData[3]);
    // Having changed config and read response, empty the buffer
    this->emptyBuffer();
    return true;
}

bool e220Module::getModuleInformation(struct ModuleInfo *info) {
    ModuleMode prevMode = this->_mode;
    ModuleStatus res = setMode(E220_MODE_PROGRAM);
    RETURN_AND_SET(res)

    res = this->sendCommand(CMD_CFG_READ, REG_MODULE_INFO, sizeof(struct ModuleInfo));
    if (res != E220_SUCCESS) {
#ifdef E220_DEBUG
        Serial.print("getModuleInformation: Failed to send command: ");
        Serial.println(res);
#endif /* E220_DEBUG */
        this->setMode(prevMode);
        return res;
    }

    uint8_t moduleData[6];
    res = this->receiveBytes(&moduleData[0], sizeof(moduleData));
    this->setMode(prevMode);

    RETURN_AND_SET(res)

    if (moduleData[0] != CMD_CFG_READ || moduleData[1] != REG_MODULE_INFO || moduleData[2] != 3) {
        return false;
    }
    memcpy(info, &moduleData[3], 3);
    return true;
}

bool e220Module::receiveMessages(ModuleMessage *msg) {
    size_t msgLen = stream->available();
    uint8_t *tmp = (uint8_t *)malloc(msgLen);

    ModuleStatus res = receiveBytes(tmp, msgLen);
    RETURN_AND_SET(res)

    if (_rssiEnable)
        msg->rssi = tmp[--msgLen];

    msg->msgLen = msgLen;
    msg->msg = tmp;
    return true;
}

bool e220Module::receiveMessageSized(ModuleMessage *msg, size_t rqdLen) {
    if (stream->available() < rqdLen)
        return false;
    size_t msgLen = rqdLen;
    if (_rssiEnable)
        msgLen++;
    uint8_t *tmp = (uint8_t *)malloc(msgLen);

    ModuleStatus res = receiveBytes(tmp, msgLen);
    RETURN_AND_SET(res)

    if (_rssiEnable)
        msg->rssi = tmp[--msgLen];

    msg->msgLen = msgLen;
    msg->msg = tmp;
    return true;
}

bool e220Module::sendMessage(const uint8_t *msg, const size_t msgLen) {
    ModuleStatus res;
    if (_fixedTx) {
#ifdef E220_DEBUG
        Serial.print("Sending via Fixed Address: ");
        Serial.print(_address);
        Serial.print(" @ channel ");
        Serial.println(_channel);
#endif /* E220_DEBUG */
        res = fixedSendBytes(_address, _channel, msg, msgLen);
    } else {
        res = sendBytes(msg, msgLen);
    }
    _lastError = res;
    return res == E220_SUCCESS;
}

bool e220Module::sendMessage(const String msg) {
    ModuleStatus res;
    if (_fixedTx) {
#ifdef E220_DEBUG
        Serial.print("Sending via Fixed Address: ");
        Serial.print(_address);
        Serial.print(" @ channel ");
        Serial.println(_channel);
#endif /* E220_DEBUG */
        res = fixedSendBytes(_address, _channel, (const uint8_t *)msg.c_str(), msg.length());
    } else {
        res = sendBytes((const uint8_t *)msg.c_str(), msg.length());
    }
    _lastError = res;
    return res == E220_SUCCESS;
}

bool e220Module::sendFixedMessage(const uint16_t addr, const uint8_t chan, const String msg) {
    ModuleStatus res = fixedSendBytes(addr, chan, (uint8_t *)msg.c_str(), msg.length());
    _lastError = res;
    return res == E220_SUCCESS;
}

bool e220Module::sendFixedMessage(const uint16_t addr, const uint8_t chan, const uint8_t *msg, const size_t msgLen) {
    ModuleStatus res = fixedSendBytes(addr, chan, msg,  msgLen);
    _lastError = res;
    return res == E220_SUCCESS;
};

bool e220Module::sendFixedBroadcastMessage(const uint8_t chan, const String msg) {
    ModuleStatus res = fixedSendBytes(0xffff, chan, (const uint8_t *)msg.c_str(), msg.length());
    _lastError = res;
    return res == E220_SUCCESS;
}

bool e220Module::sendFixedBroadcastMessage(const uint8_t chan, const uint8_t *msg, const size_t msgLen) {
    ModuleStatus res = fixedSendBytes(0xffff, chan, msg,  msgLen);
    _lastError = res;
    return res == E220_SUCCESS;
};

ModuleStatus e220Module::sendCommand(uint8_t cmd, uint8_t addr, size_t length) {
	uint8_t CMD[3] = {cmd, addr, length};
#ifdef E220_DEBUG_2
    printHexBytes("sendCommand", CMD, 3);
#endif /* E220_DEBUG_2 */
	size_t size = this->stream->write(CMD, 3);
	managedDelay(50);
    return size != 3 ? E220_DATA_SIZE_ERROR : E220_SUCCESS;
}

ModuleStatus e220Module::sendBytes(const uint8_t *data, const size_t length) {
#ifdef E220_DEBUG_2
    printHexBytes("sendBytes", data, length);
#endif /* E220_DEBUG_2 */
    if (_worListener) {
        if (!setMode(E220_MODE_NORMAL)) {
#ifdef E220_DEBUG
            Serial.println("Unable to swap modes to NORMAL for transmission...");
#endif /* E220_DEBUG */
            return _lastError;
        }
    }
    size_t written = this->stream->write(data, length);
    if (_worListener) {
        if (!setMode(E220_MODE_WOR_RX)) {
#ifdef E220_DEBUG
            Serial.println("Unable to swap mode back to WOR RX");
#endif /* E220_DEBUG */
            return _lastError;
        }
    }

    if (written != length) {
#ifdef E220_DEBUG
        Serial.print("Invalid write of data: written ");
        Serial.print(written);
        Serial.print(" vs wanted ");
        Serial.println(length);
#endif /* E220_DEBUG */
        return written == 0 ? E220_NO_RESPONSE_FROM_DEVICE : E220_DATA_SIZE_ERROR;
    }
    return this->waitCompleteResponse(5000, 5000);
}

ModuleStatus e220Module::fixedSendBytes(const uint16_t addr, const uint8_t chan, const uint8_t *msg, const size_t msgLen) {
    if (! _fixedTx)
        return E220_NOT_FIXED;
    uint8_t *fixedBytes = (uint8_t *)malloc( 3 + msgLen);
    fixedBytes[0] = addr >> 8;
    fixedBytes[1] = addr & 0xff;
    fixedBytes[2] = chan;
    memcpy(&fixedBytes[3], msg, msgLen);
    ModuleStatus res = sendBytes(fixedBytes, msgLen + 3);
    free(fixedBytes);
    return res;
}

ModuleStatus e220Module::receiveBytes(uint8_t *buffer, size_t length) {
    size_t rcv = this->stream->readBytes(buffer, length);
    if (rcv != length) {
#ifdef E220_DEBUG
        Serial.print("Invalid read of data: read ");
        Serial.print(rcv);
        Serial.print(" vs wanted ");
        Serial.println(length);
#endif /* E220_DEBUG */
        return rcv == 0 ? E220_NO_RESPONSE_FROM_DEVICE : E220_DATA_SIZE_ERROR;
    }
#ifdef E220_DEBUG_2
    printHexBytes("receiveBytes", buffer, length);
#endif /* E220_DEBUG_2 */
    return this->waitCompleteResponse(1000, 100);
} 

ModuleStatus e220Module::waitCompleteResponse(unsigned long timeout, unsigned int waitNoAux = 100) {
	ModuleStatus result = E220_SUCCESS;

	unsigned long t = millis();
	if (((unsigned long) (t + timeout)) == 0){
		t = 0;
	}
	if (auxPin != -1) {
		while (digitalRead(auxPin) == LOW) {
			if ((millis() - t) > timeout){
#ifdef E220_DEBUG
				Serial.println("waitCompleteResponse: Timeout!");
#endif /* E220_DEBUG */
                return E220_TIMEOUT;
			}
            // Avoid WDT Reset
            yield();
		}
	} else {
		managedDelay(waitNoAux);
    }

#ifdef E220_DEBUG_2
	Serial.println("waitCompleteResponse: Complete");
#endif /* E220_DEBUG_2 */

	// Per data sheet, AUX should go high for 2ms before any other controls can be issued, so wait
    // at least that long.
	managedDelay(20);
	return result;
}
