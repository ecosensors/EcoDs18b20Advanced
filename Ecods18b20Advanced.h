

#ifndef Ecods18b20_h
#define Ecods18b20_h

#include <OneWire.h>

// Model IDs
#define DS18S20MODEL 0x10  // also DS1820
#define DS18B20MODEL 0x28  // also MAX31820
#define DS1822MODEL  0x22
#define DS1825MODEL  0x3B
#define DS28EA00MODEL 0x42

// Error Codes
#define DEVICE_DISCONNECTED_C -127
#define DEVICE_DISCONNECTED_F -196.6
#define DEVICE_DISCONNECTED_RAW -7040

typedef uint8_t DeviceAddress[8];

class Ecods18b20{
 	protected:

 	private:
 		bool _useExternalPullup;
 		OneWire* _wire;
 		// count of devices on the bus
		uint8_t _devices;
		// count of DS18xxx Family devices on bus
		uint8_t _ds18Count;
		// parasite power on or off
		bool _parasite;
		// used to determine the delay amount needed to allow for the
		// temperature conversion to take place
		uint8_t _bitResolution;
		// used to requestTemperature with or without delay
		bool _waitForConversion;
		// used to requestTemperature to dynamically check if a conversion is complete
		bool _checkForConversion;
		 // used to determine if values will be saved from scratchpad to EEPROM on every scratchpad write
  		bool _autoSaveScratchPad;
  		typedef uint8_t ScratchPad[9];

  		// Returns true if all bytes of scratchPad are '\0'
		bool _isAllZeros(const uint8_t* const scratchPad, const size_t length = 9);
		// External pullup control
    	void _activateExternalPullup(void);
    	void _deactivateExternalPullup(void);
    	uint8_t pullupPin;
    	// reads scratchpad and returns the raw temperature
		int16_t _calculateTemperature(const uint8_t*, uint8_t*);
	
	public:
		Ecods18b20();
		void setOneWire(OneWire*);
		// initialise bus
		void begin(void);
		// returns true if address is valid
		bool validAddress(const uint8_t*);
		// returns true if address is of the family of sensors the lib supports.
		bool validFamily(const uint8_t* deviceAddress);
		// read device's power requirements
		bool readPowerSupply(const uint8_t* deviceAddress = nullptr);
		// get global resolution
		uint8_t getResolution();
		// returns the device resolution: 9, 10, 11, or 12 bits
		uint8_t getResolution(const uint8_t*);
		// attempt to determine if the device at the given address is connected to the bus
		bool isConnected(const uint8_t*);
		// attempt to determine if the device at the given address is connected to the bus
		// also allows for updating the read scratchpad
		bool isConnected(const uint8_t*, uint8_t*);
		// read device's scratchpad
		bool readScratchPad(const uint8_t*, uint8_t*);
		// finds an address at a given index on the bus
		bool getAddress(uint8_t*, uint8_t);
		// set global resolution to 9, 10, 11, or 12 bits
		void setResolution(uint8_t);
		// set resolution of a device to 9, 10, 11, or 12 bits
		bool setResolution(const uint8_t*, uint8_t, bool skipGlobalBitResolutionCalculation = false);
		// write device's scratchpad
		void writeScratchPad(const uint8_t*, const uint8_t*);
		// Sends command to one or more devices to save values from scratchpad to EEPROM
  		// Returns true if no errors were encountered, false indicates failure
  		bool saveScratchPad(const uint8_t* = nullptr);
		// sends command for all devices on the bus to perform a temperature conversion
		void requestTemperatures(void);
		// sends command for one device to perform a temperature conversion by address
		bool requestTemperaturesByAddress(const uint8_t*);
		// sends command for one device to perform a temperature conversion by index
		bool requestTemperaturesByIndex(uint8_t);
		// Get temperature for device index (slow)
		float getTempCByIndex(uint8_t);
		// Get temperature for device index (slow)
		float getTempFByIndex(uint8_t);
		// returns temperature in degrees C
		float getTempC(const uint8_t*);
		// returns temperature in degrees F
		float getTempF(const uint8_t*);
		// returns temperature raw value (12 bit integer of 1/128 degrees C)
		int16_t getTemp(const uint8_t*);

		void blockTillConversionComplete(uint8_t);
		// convert from raw to Celsius
		static float rawToCelsius(int16_t);
		// convert from raw to Fahrenheit
		static float rawToFahrenheit(int16_t);
		// Is a conversion complete on the wire? Only applies to the first sensor on the wire.
		bool isConversionComplete(void);
		static uint16_t millisToWaitForConversion(uint8_t);

 };
 #endif