#include "e220.h"

typedef struct {
    int pub;
    uint8_t priv;
} bitOption;
#define STRUCT_ARRAY_SZ(arr) (sizeof(arr) / sizeof(arr[0]))


void e220Module::parseConfigurationBytes(uint8_t *data) {
    _address = (data[0] << 8 | data[1]);
    _uartBaudRate = data[2] >> 5;
    _uartParity = (data[2] >> 3) & 0x03;
    _airDataRate = data[2] & 0x07;
    _txPower = data[3] & 0x03;
    _rssiNoise = (data[3] & 0x20) == 0x20;
	_sps = data[3] >> 6;
    _channel = data[4];
    _worPeriod = data[5] & 0x07;
    _lbtEnable = (data[5] & 0x10) == 0x10;
    _fixedTx = (data[5] & 0x40) == 0x40;
    _rssiEnable = (data[5] &  0x80) == 0x80;
}

uint8_t *e220Module::buildConfigurationBytes() {
    uint8_t *rv = (uint8_t *)calloc(6, sizeof(uint8_t));
    rv[0] = _address >> 8;
    rv[1] = _address & 0xff;
    rv[2] = (_uartBaudRate << 5) | (_uartParity << 3) | _airDataRate; 
    rv[3] = _txPower;
    if (_rssiNoise)
        rv[3] |= (1 << 5);
    rv[3] |= (_sps << 6);
    rv[4] = _channel;
    rv[5] = _worPeriod;
    if (_lbtEnable)
        rv[5] |= (1 << 4);
    if (_fixedTx)
        rv[5] |= (1 << 6);
    if (_rssiEnable)
        rv[5] |= (1 << 7);

    return rv;
}

static bitOption parityOpts[] = {
    {SERIAL_8N1, 0b00},
    {SERIAL_8E1, 0b10},
    {SERIAL_8O1, 0b01},
    {SERIAL_8N1, 0b11},
};

bool e220Module::setSerialParity(int val) {
    uint8_t current = _uartParity;
    bool found = false;
    for (int i = 0; i < STRUCT_ARRAY_SZ(parityOpts); i++) {
        if (parityOpts[i].pub == val) {
            _uartParity = parityOpts[i].priv;
            found = true;
            break;
        }
    }
    if (found) {
        _dirty |= (current != _uartParity);
    }
#ifdef E220_DEBUG
     else {
        Serial.println("Unable to set the UART Parity requested. Options are 8N1, 8E1 or 8O1");
    }
#endif /* E220_DEBUG */
    return found;
}

int e220Module::getSerialParity() {
    for (int i = 0; i < STRUCT_ARRAY_SZ(parityOpts); i++) {
        if (parityOpts[i].priv == _uartParity)
            return parityOpts[i].pub;
    }
    return -1;
}

static bitOption serialSpeeds[] = {
    {1200, 0b000},
    {2400, 0b001},
    {4800, 0b010},
    {9600, 0b011},
    {19200, 0b100},
    {38400, 0b101},
    {57600, 0b110},
    {115200, 0b111}
};

bool e220Module::setSerialSpeed(int rate) {
    uint8_t current = _uartBaudRate;
    bool found = false;
    for (int i = 0; i < STRUCT_ARRAY_SZ(serialSpeeds); i++) {
        if (serialSpeeds[i].pub == rate) {
            _uartBaudRate = serialSpeeds[i].priv;
            found = true;
            break;
        }
    }
    if (found) {
        _dirty |= (current != _uartBaudRate);
    }
#ifdef E220_DEBUG
     else {
        Serial.println("Invalid value for Serial Speed. Possible values: 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200");
    }
#endif /* E220_DEBUG */
    return found;
}

int e220Module::getSerialSpeed() {
    for (int i = 0; i < STRUCT_ARRAY_SZ(serialSpeeds); i++) {
        if (serialSpeeds[i].priv == _uartBaudRate)
            return serialSpeeds[i].pub;
    }
    return -1;
}

static bitOption airDataRates[] = {
    {2400, 0b010},
    {4800, 0b011},
    {9600, 0b100},
    {19200, 0b101},
    {38400, 0b110},
    {62500, 0b111}
};

bool e220Module::setAirDataRate(int rate) {
    uint8_t current = _airDataRate;
    bool found = false;
    for (int i = 0; i < STRUCT_ARRAY_SZ(airDataRates); i++) {
        if (airDataRates[i].pub == rate) {
            _airDataRate = airDataRates[i].priv;
            found = true;
            break;
        }
    }
    if (found) {
        _dirty |= (current != _airDataRate);
    }
#ifdef E220_DEBUG
     else {
        Serial.println("Invalid value for Air Data Rate. Possible values: 2400, 4800, 9600, 19200, 38400, 62500");
    }
#endif /* E220_DEBUG */
    return found;
}

int e220Module::getAirDataRate() {
    for (int i = 0; i < STRUCT_ARRAY_SZ(airDataRates); i++) {
        if (airDataRates[i].priv == _airDataRate)
            return airDataRates[i].pub;
    }
    return -1;
}

static bitOption worPeriods[] = {
    {500, 0b000},
	{1000, 0b001},
	{1500, 0b010},
	{2000, 0b011},
	{2500, 0b100},
	{3000, 0b101},
	{3500, 0b110},
	{4000, 0b111}
};

bool e220Module::setWORPeriod(int rate) {
    uint8_t current = _worPeriod;
    bool found = false;
    for (int i = 0; i < STRUCT_ARRAY_SZ(worPeriods); i++) {
        if (worPeriods[i].pub == rate) {
            _worPeriod = worPeriods[i].priv;
            found = true;
            break;
        }
    }
    if (found) {
        _dirty |= (current != _worPeriod);
    }
#ifdef E220_DEBUG
     else {
        Serial.println("Invalid value for WOR Period. Possible values: 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000.");
    }
#endif /* E220_DEBUG */
    return found;
}

int e220Module::getWORPeriod() {
    for (int i = 0; i < STRUCT_ARRAY_SZ(worPeriods); i++) {
        if (worPeriods[i].priv == _worPeriod)
            return worPeriods[i].pub;
    }
    return -1;
}

static bitOption spss[] = {
    {200, 0b00},
    {128, 0b01},
    {64, 0b10},
    {32, 0b11}
};

bool e220Module::setSubPacking(int rate) {
    uint8_t current = _sps;
    bool found = false;
    for (int i = 0; i < STRUCT_ARRAY_SZ(spss); i++) {
        if (spss[i].pub == rate) {
            _sps = spss[i].priv;
            found = true;
            break;
        }
    }
    if (found) {
        _dirty |= (current != _sps);
    }
#ifdef E220_DEBUG
     else {
        Serial.println("Invalid value for Sub Packing. Possible values: 200, 128, 64, 32.");
    }
#endif /* E220_DEBUG */
    return found;
}

int e220Module::getSubPacking() {
    for (int i = 0; i < STRUCT_ARRAY_SZ(spss); i++) {
        if (spss[i].priv == _sps)
            return spss[i].pub;
    }
    return -1;
}

static bitOption txPwrs[] = {
    {22, 0b00},
    {17, 0b01},
    {13, 0b10},
    {10, 0b11}
};

bool e220Module::setTxPower(int rate) {
    uint8_t current = _txPower;
    bool found = false;
    for (int i = 0; i < STRUCT_ARRAY_SZ(txPwrs); i++) {
        if (txPwrs[i].pub == rate) {
            _sps = txPwrs[i].priv;
            found = true;
            break;
        }
    }
    if (found) {
        _dirty |= (current != _txPower);
    }
#ifdef E220_DEBUG    
     else {
        Serial.println("Invalid value for Tx Power. Possible values: 22, 17, 13, 10.");
    }
#endif /* E220_DEBUG */
    return found;
}

int e220Module::getTxPower() {
    for (int i = 0; i < STRUCT_ARRAY_SZ(txPwrs); i++) {
        if (txPwrs[i].priv == _txPower)
            return txPwrs[i].pub;
    }
    return -1;
}

bool e220Module::configurationReset(bool save) {
    _address = 0x000c;
    _channel = 0; /* No default listed on datasheet */
    _uartBaudRate = 0b011;
    _uartParity = 0b00;
    _airDataRate = 0b010;
    _sps = 0b00;
    _rssiNoise = false;
    _txPower = 0b00;
    _rssiEnable = false;
    _fixedTx = false;
    _lbtEnable = false;
    _worPeriod = 0b011; /* No default listed on datasheet */
    return setModuleConfiguration(save);
}

 String e220Module::getLastErrorDescription() {
    switch(_lastError) {
        case E220_OK:
        case E220_SUCCESS:
            return F("OK");
        case E220_UNKNOWN:
            return F("Unknown error???");
        case E220_NOT_SUPPORT:
            return F("Operation not supported");
        case E220_NOT_IMPLEMENT:
            return F("Operation not implemented");
        case E220_INVALID_PARAM:
            return F("One or more parameters are invalid");
        case E220_DATA_SIZE_ERROR:
            return F("Data size was not as required");
        case E220_BUF_TOO_SMALL:
            return F("Buffer was too small");
        case E220_TIMEOUT:
            return F("Timed out");
        case E220_HARDWARE:
            return F("Hardware error");
        case E220_NO_RESPONSE_FROM_DEVICE:
            return F("No response from denive. Check connection and pin setup");
        case E220_WRONG_FORMAT:
            return F("Wrong format returned from module");
        case E220_PACKET_TOO_BIG:
            return F("Packet is too big");
        case E220_NOT_FIXED:
            return F("Attempt to send to a fixed address without being in fixed tranmission mode. Use setFixed(true)");
        case E220_MODE_ERROR:
            return F("Unable to change mode as required");
    }
    return F("Unknown error code? Ruh roh.");
 }

static void printHex(uint8_t n) {
    Serial.print(n < 16 ? "0" : "");
    Serial.print(n, HEX);
}

void e220Module::printConfiguration() {
    Serial.println("----------------------------------------");
    Serial.println("       Current Configuration");
    Serial.println("----------------------------------------");

    if (_dirty) {
        Serial.println("  ****\n  **** Configuration is NOT the same as E220 Module.\n  ****");
    }
    Serial.print(F("Address : 0x"));
    printHex((_address >> 8));
    printHex((_address & 0xFF));
    Serial.println();

    Serial.print(F("Channel : 0x"));
    printHex(_channel);
    Serial.print(" [ ");
    Serial.print(_channel);
    Serial.print(" ] @ ");
    Serial.print(getFrequency());
    Serial.println("MHz");
    Serial.print("Serial: Parity : ");
    Serial.println(getSerialParity());
    Serial.print("        Speed  : ");
    Serial.print(getSerialSpeed());
    Serial.println("bps");

    Serial.print(F("Air Data Rate  : "));
    Serial.print(getAirDataRate());
    Serial.println("bps");

    Serial.print(F("Sub Packet Size: "));
    Serial.print(getSubPacking());
    Serial.println(" bytes");
    Serial.println("---------------");
    Serial.print(F("Transmit: Power             : "));
    Serial.print(getTxPower());
    Serial.println("db");
    Serial.print(F("          WOR Period        : "));
    Serial.print(getWORPeriod());
    Serial.println("ms");
    Serial.print(F("          Mode              : "));
    Serial.println(_fixedTx ? "Fixed" : "Transparent");
    Serial.print(F("          LBT Enabled?      "));
    Serial.println(_lbtEnable ? "Yes" : "No");
    Serial.print(F("          RSSI : Enabled?               "));
    Serial.println(_rssiEnable ? "Yes" : "No");
    Serial.print(F("               : Ambient Noise Enabled? "));  
    Serial.println(_rssiNoise ? "Yes" : "No");
    Serial.println("----------------------------------------");
}

void ModuleInfo::printDetails() {
    Serial.println("----------------------------------------");
    Serial.println("     E220 Module Information");
    Serial.println("----------------------------------------");
    Serial.print("Model Number   : 0x");
    printHex(model);
    Serial.println();
    Serial.print("Version Number : 0x");
    printHex(version);
    Serial.println();
    Serial.print("Features       : 0x");
    printHex(features);
    Serial.println();
}

#ifdef E220_DEBUG
void e220Module::printHexBytes(String prefix, const uint8_t *bytes, const size_t bytesLen) {
    Serial.print(prefix);
    Serial.print(": ");
    for (size_t i = 0; i < bytesLen; i++) {
        printHex(bytes[i]);
        Serial.print(" ");
    }
    Serial.println();
}
#endif /* E220_DEBUG */
