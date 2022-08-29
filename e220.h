/* e220.h
 *
 * Header file for eByte E220 Lora Module.
 */
#ifndef E220_H
#define E220_H

#include "Arduino.h"

//#define E220_DEBUG true
//#define E220_DEBUG_2 true

#if defined(E220_DEBUG_2) && !defined(E220_DEBUG)
#define E220_DEBUG true
#endif

enum ModuleMode {
    E220_MODE_NORMAL = 0,
    E220_MODE_WOR_TX,
    E220_MODE_WOR_RX,
    E220_MODE_PROGRAM,
    E220_MODE_UNKNOWN = 255
};

enum ModuleFrequency {
    FREQ_433 = 410,
    FREQ_868 = 850,
    FREQ_915 = 850
};

typedef enum {
    E220_OK = 0,
    E220_SUCCESS,
    E220_UNKNOWN,
    E220_NOT_SUPPORT,
    E220_NOT_IMPLEMENT,
    E220_INVALID_PARAM,
    E220_DATA_SIZE_ERROR,
    E220_BUF_TOO_SMALL,
    E220_TIMEOUT,
    E220_HARDWARE,
    E220_NO_RESPONSE_FROM_DEVICE,
    E220_WRONG_FORMAT,
    E220_PACKET_TOO_BIG,
    E220_NOT_FIXED,
    E220_MODE_ERROR
} ModuleStatus;

enum ModuleCommand {
	CMD_CFG_SAVE 	= 0xC0,
	CMD_CFG_READ	= 0xC1,
    CMD_CFG_TEMP    = 0xC2
};

enum ModuleRegister {
	REG_CFG         = 0x00,
	REG_UART        = 0x02,
	REG_TRANS_MODE,
	REG_CHANNEL,
	REG_OPTION,
	REG_CRYPT,
	REG_MODULE_INFO	= 0x08
};


struct ModuleInfo {
    uint8_t model;
    uint8_t version;
    uint8_t features;

    void printDetails();
};

/**
 * @brief ModuleMessage is a struct used to pass message data to the user.
 * 
 */
typedef struct {
    uint8_t *msg;
    size_t msgLen;
    uint8_t rssi;

    /**
     * @brief Return the message contents as a string.
     * 
     * @return String 
     */
    String asString() {
       char *buf = (char *)calloc(msgLen + 1, sizeof(char *));
        memcpy(buf, msg, msgLen);
        String s = String(buf);
        free(buf);
        return s;
    };
    /**
     * @brief Get a numeric value for the Chanel noise. This is only available if RSSI is enabled.
     * 
     * @return int16_t Noise level in dBm
     */
    int16_t channelNoise() { return -1 * (256 - rssi); }
} ModuleMessage;

class e220Module {
    public:
        e220Module(uint8_t rx, uint8_t tx);
        e220Module(uint8_t rx, uint8_t tx, uint8_t aux);
        e220Module(uint8_t rx, uint8_t tx, uint8_t aux, uint8_t m0, uint8_t m1);

        bool begin(int baudRate);

        /**
         * @brief Set the currently held configuration into the E220 module. Module Configuration object
         * 
         * @param save Write the configuration into permanent memory if true. Otherwise write as temporary.
         * @return true   Configuration has been saved.
         * @return false  Configuration could not be written. getLastError() will show the error code.
         */
        bool setModuleConfiguration(bool save);
        /**
         * @brief Get the Module Configuration from the E220 module 
         * 
         * @return true Configuration has been loaded.
         * @return false Unable to get the module configuration. getLastError() will show the error code.
         */
        bool getModuleConfiguration();
        /**
         * @brief Print the currently held configuration to Serial.
         * 
         */
        void printConfiguration();
        /**
         * @brief Check whether the stored configuration is different to that currently being used by the E220 Module.
         * 
         * @return true 
         * @return false 
         */
        bool hasConfigurationChanged() { return _dirty; }
        /**
         * @brief Check whether the E220 module configuration has been retrieved.
         * 
         * @return true 
         * @return false 
         */
        bool hasConfiguration() { return _channel != -1; }
        /**
         * @brief Reset the current configuration to a known state within the E220 Module.
         * 
         * This resets the current configuration but also saves it to the module. There is no need to
         * call setModuleConfiguration() after this function.
         * 
         * @param save Save the configuration permanently if true.
         * @return true 
         * @return false 
         */
        bool configurationReset(bool save);

        /**
         * @brief Get the Module Information object
         * 
         * @param info POinter to the ModuleInfo object to record the information.
         * @return true Module information was retrieved OK.
         * @return false Unable to get the module information. getLastError() will return error code.
         */
        bool getModuleInformation(ModuleInfo *info);

        // Buffer functions
        /**
         * @brief Check whether there is any data to be retrieved from the buffer.
         * This works for WOR operations as well.
         * 
         * @return true Data is available / WOR signal has been received.
         * @return false 
         */
        bool available();

        void flush() { stream->flush(); }
        void emptyBuffer() { while (stream->available()) { this->stream->read(); }}

        // Configuration
        /**
         * @brief Get the current address as an unsigned 16 bit value.
         * 
         * @return uint16_t 
         */
        uint16_t getAddress() { return _address; }
        /**
         * @brief Set the address to transmit
         * 
         * @param addr uint16_t address
         */
        void setAddress(uint16_t addr) { _dirty |= (addr != _address); _address = addr; };
        /**
         * @brief Set the address to the broadcast value, 0xFFFF
         * 
         */
        void setBroadcast() { setAddress(0xffff); }

        /**
         * @brief Get configured channel number.
         * @return Channel number or -1 if it is not available.
         */
        uint8_t getChannel() { return _channel; }
        /**
         * @brief Set the Channel object
         * 
         * @param chan Channel number to be set.
         */
        void setChannel(uint8_t chan) { _dirty |= (chan != _channel); _channel = chan; };
        /**
         * @brief Get the frequency of the currently tuned channel.
         * 
         * @return float Frequency in MHz. Return 0.0 if no channel tuning information available.
         */
        float getFrequency() { return _channel == -1 ? 0 : this->freqBase + this->_channel + .125; }

        /**
         * @brief Is RSSI enabled?
         * 
         * @return true RSSI is enabled.
         * @return false RSSI is not enabled. (Default)
         */
        bool isRSSI() { return _rssiEnable; }
        /**
         * @brief Enable/Disable RSSI.
         * 
         * @param onoff Bolean to indicate whether to enable/disable RSSI 
         * @return true RSSI is enabled.
         * @return false RSSI is not enabled.
         */
        bool setRSSI(bool onoff) { _dirty |= (onoff != _rssiEnable); _rssiEnable = onoff; };

        /**
         * @brief Set the Fixed transmission flag. When set the sendFixed* functions can be used.
         * 
         * @param onoff Whether to enable or disable.
         * @return true 
         * @return false 
         */
        bool setFixed(bool onoff) { _dirty |= (onoff != _fixedTx); _fixedTx = onoff; };
        /**
         * @brief Is the present configuration set for fixed transmission?
         * 
         * @return true 
         * @return false 
         */
        bool isFixed() { return _fixedTx; }

        bool lbtEnabled() { return _lbtEnable; }
        bool setLBT(bool onoff) { _dirty |= (onoff != _lbtEnable); _lbtEnable = onoff; };

        bool setSerialParity(int val);
        int getSerialParity();
        String getSerialDescription();

        bool setSerialSpeed(int val);
        int getSerialSpeed();

        bool setAirDataRate(int val);
        int getAirDataRate();

        bool setSubPacking(int val);
        int getSubPacking();

        bool setTxPower(int val);
        int getTxPower();

        bool setWORPeriod(int val);
        int getWORPeriod();

        uint16_t setEncyptionKey(uint16_t);

        bool receiveMessages(ModuleMessage *msg);
        bool receiveMessageSized(ModuleMessage *msg,size_t msgLen);

        bool sendMessage(const uint8_t *msg, const size_t msgLen);
        bool sendMessage(const String msg);

        bool sendFixedMessage(const uint16_t addr, const uint8_t chan, const String msg);
        bool sendFixedMessage(const uint16_t addr, const uint8_t chan, const uint8_t *msg, const size_t msgLen);

        bool sendFixedBroadcastMessage(const uint8_t chan, const String msg);
        bool sendFixedBroadcastMessage(const uint8_t chan, const uint8_t *msg, const size_t msgLen);

        bool setupWORListener(int period);
        bool stopWORListener();
        bool setupWORSender(int period);
        /**
         * @brief Check whether a WOR signal has been received. Only valid if set as a WOR listener.
         * This is deprecated as calling avalable() will now provide a simpler solution.
         * 
         * @return true WOR Signal has been received.
         * @return false 
         */
        bool checkWOR();
        /**
         * @brief Reset the WOR signal. This should be called prior to attempting to wait for a subsequent signal.
         * 
         */
        void resetWOR();

        ModuleStatus getLastError() { return _lastError; };
        String getLastErrorDescription();
    private:
        // Internal Configuration
        uint8_t rxPin  = -1;
        uint8_t txPin  = -1;
        uint8_t auxPin = -1;
        uint8_t m0Pin  = -1;
        uint8_t m1Pin  = -1;

        // e220 Configuration is 6 bytes of information.
        uint16_t _address = -1;  // bytes 0 & 1
        // byte 2
        uint8_t _airDataRate;       // bits 0-2
	    uint8_t _uartParity;        // bits 3-4
	    uint8_t _uartBaudRate;      // bits 5-7
        // byte 3
	    uint8_t _txPower;           //bits 0-1
	    uint8_t _rssiNoise;         //bit 5
	    uint8_t _sps;               //bit 6-7
        // byte 4
        uint8_t _channel = -1;
        // byte 5
        uint8_t  _worPeriod;        //bit 0-2
	    bool _lbtEnable;            //bit 4
        bool _fixedTx;              //bit 6
	    bool _rssiEnable;           //bit 7

        // e220 Operation
        ModuleMode _mode = E220_MODE_UNKNOWN;
        ModuleStatus _lastError = E220_OK;

        bool _dirty = false;
        
        bool _worListener = false;

        Stream *stream = NULL;

        ModuleFrequency freqBase = FREQ_868;

        bool hasAux() { return this->auxPin != -1; }
        bool canSetModes() { return (this->m0Pin != -1 && this->m1Pin != -1); }

        uint8_t *buildConfigurationBytes();
        void parseConfigurationBytes(uint8_t *data);

        ModuleStatus setMode(ModuleMode mode);

        ModuleStatus readConfig();
        ModuleStatus sendCommand(uint8_t cmd, uint8_t reg, size_t length);
        ModuleStatus sendBytes(const uint8_t *data, const size_t length);
        ModuleStatus fixedSendBytes(const uint16_t addr, const uint8_t chan, const uint8_t *msg, const size_t msgLen);
        ModuleStatus receiveBytes(uint8_t *buffer, size_t length);
        ModuleStatus waitCompleteResponse(unsigned long timeout, unsigned int waitNoAux);

#ifdef E220_DEBUG_2
        void printHexBytes(String prefix, const uint8_t *bytes, const size_t bytesLen);
#endif/* E220_DEBUG */
};

#endif /* E220_H */
