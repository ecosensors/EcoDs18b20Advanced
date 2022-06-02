/*
* EcoSensors
* Library is still under developpent
* 
*/

#include "Arduino.h"
#include "Ecods18b20Advanced.h"


// DSROM FIELDS
#define DSROM_FAMILY    0
#define DSROM_CRC       7

// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy scratchpad to EEPROM
#define READSCRATCH     0xBE  // Read from scratchpad
#define WRITESCRATCH    0x4E  // Write to scratchpad
#define RECALLSCRATCH   0xB8  // Recall from EEPROM to scratchpad
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

#define MAX_CONVERSION_TIMEOUT		750


Ecods18b20::Ecods18b20() {
	_useExternalPullup = false;
}

void Ecods18b20::setOneWire(OneWire* _oneWire)
{
	_wire = _oneWire;
	_devices = 0;
	_ds18Count = 0;
	_parasite = false;
	_bitResolution = 9;
	_waitForConversion = true;
	_checkForConversion = true;
  	_autoSaveScratchPad = true;
}

// initialise the bus
void Ecods18b20::begin(void) {

	DeviceAddress deviceAddress;

	_wire->reset_search();
	_devices = 0; // Reset the number of devices when we enumerate wire devices
	_ds18Count = 0; // Reset number of DS18xxx Family devices

	while (_wire->search(deviceAddress))
	{
		if (validAddress(deviceAddress))
		{
			_devices++;
			if (validFamily(deviceAddress))
			{
				_ds18Count++;
				if (!_parasite && readPowerSupply(deviceAddress))
					_parasite = true;

				uint8_t b = getResolution(deviceAddress);
				if (b > _bitResolution) _bitResolution = b;
			}
		}
	}
}

bool Ecods18b20::validAddress(const uint8_t* deviceAddress) {
	return (_wire->crc8(deviceAddress, 7) == deviceAddress[DSROM_CRC]);
}

bool Ecods18b20::validFamily(const uint8_t* deviceAddress) {
	switch (deviceAddress[DSROM_FAMILY])
	{
		case DS18S20MODEL:
		case DS18B20MODEL:
		case DS1822MODEL:
		case DS1825MODEL:
		case DS28EA00MODEL:
			return true;
		default:
			return false;
	}
}

// returns true if parasite mode is used (2 wire)
// returns false if normal mode is used (3 wire)
// if no address is given (or nullptr) it checks if any device on the bus
// uses parasite mode.
// See issue #145
bool Ecods18b20::readPowerSupply(const uint8_t* deviceAddress)
{
	bool parasiteMode = false;
	_wire->reset();
	if (deviceAddress == nullptr)
		_wire->skip();
	else
		_wire->select(deviceAddress);

	_wire->write(READPOWERSUPPLY);
	if (_wire->read_bit() == 0)
		parasiteMode = true;

	_wire->reset();
	return parasiteMode;
}

// returns the current resolution of the device, 9-12
// returns 0 if device not found
uint8_t Ecods18b20::getResolution(const uint8_t* deviceAddress) {
	// DS1820 and DS18S20 have no resolution configuration register
	if (deviceAddress[DSROM_FAMILY] == DS18S20MODEL)
		return 12;

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad))
	{
		switch (scratchPad[CONFIGURATION])
		{
			case TEMP_12_BIT:
				return 12;
			case TEMP_11_BIT:
				return 11;
			case TEMP_10_BIT:
				return 10;
			case TEMP_9_BIT:
				return 9;
		}
	}
	return 0;
}

// attempt to determine if the device at the given address is connected to the bus
bool Ecods18b20::isConnected(const uint8_t* deviceAddress) {

	ScratchPad scratchPad;
	return isConnected(deviceAddress, scratchPad);

}

// attempt to determine if the device at the given address is connected to the bus
// also allows for updating the read scratchpad
bool Ecods18b20::isConnected(const uint8_t* deviceAddress,uint8_t* scratchPad)
{
	bool b = readScratchPad(deviceAddress, scratchPad);
	return b && !_isAllZeros(scratchPad) && (_wire->crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);
}

bool Ecods18b20::readScratchPad(const uint8_t* deviceAddress, uint8_t* scratchPad)
{

	// send the reset command and fail fast
	int b = _wire->reset();
	if (b == 0)
		return false;

	_wire->select(deviceAddress);
	_wire->write(READSCRATCH);

	// Read all registers in a simple loop
	// byte 0: temperature LSB
	// byte 1: temperature MSB
	// byte 2: high alarm temp
	// byte 3: low alarm temp
	// byte 4: DS18S20: store for crc
	//         DS18B20 & DS1822: configuration register
	// byte 5: internal use & crc
	// byte 6: DS18S20: COUNT_REMAIN
	//         DS18B20 & DS1822: store for crc
	// byte 7: DS18S20: COUNT_PER_C
	//         DS18B20 & DS1822: store for crc
	// byte 8: SCRATCHPAD_CRC
	for (uint8_t i = 0; i < 9; i++) {
		scratchPad[i] = _wire->read();
	}

	b = _wire->reset();
	return (b == 1);
}

// Returns true if all bytes of scratchPad are '\0'
bool Ecods18b20::_isAllZeros(const uint8_t * const scratchPad, const size_t length)
{
	for (size_t i = 0; i < length; i++)
	{
		if (scratchPad[i] != 0)
		{
			return false;
		}
	}

	return true;
}

// finds an address at a given index on the bus
// returns true if the device was found
bool Ecods18b20::getAddress(uint8_t* deviceAddress, uint8_t index) {

	uint8_t depth = 0;

	_wire->reset_search();

	while (depth <= index && _wire->search(deviceAddress)) {
		if (depth == index && validAddress(deviceAddress))
			return true;
		depth++;
	}

	return false;

}

// set resolution of all devices to 9, 10, 11, or 12 bits
// if new resolution is out of range, it is constrained.
void Ecods18b20::setResolution(uint8_t newResolution)
{
	_bitResolution = constrain(newResolution, 9, 12);
	DeviceAddress deviceAddress;
	for (uint8_t i = 0; i < _devices; i++)
	{
		getAddress(deviceAddress, i);
		setResolution(deviceAddress, _bitResolution, true);
	}
}

// set resolution of a device to 9, 10, 11, or 12 bits
// if new resolution is out of range, 9 bits is used.
bool Ecods18b20::setResolution(const uint8_t* deviceAddress, uint8_t newResolution, bool skipGlobalBitResolutionCalculation)
{

 	bool success = false;

  	// DS1820 and DS18S20 have no resolution configuration register
  	if (deviceAddress[DSROM_FAMILY] == DS18S20MODEL)
  	{
    	success = true;
  	}
  	else
  	{
  		// handle the sensors with configuration register
 		newResolution = constrain(newResolution, 9, 12);
  		uint8_t newValue = 0;
  		ScratchPad scratchPad;

  		// we can only update the sensor if it is connected
  		if (isConnected(deviceAddress, scratchPad))
  		{
  			switch (newResolution)
  			{
    			case 12:
    				newValue = TEMP_12_BIT;
    				break;
        		case 11:
        			newValue = TEMP_11_BIT;
        			break;
        		case 10:
        			newValue = TEMP_10_BIT;
        			break;
        		case 9:
        		default:
        			newValue = TEMP_9_BIT;
        			break;
    		}

			// if it needs to be updated we write the new value
			if (scratchPad[CONFIGURATION] != newValue)
			{
				scratchPad[CONFIGURATION] = newValue;
				writeScratchPad(deviceAddress, scratchPad);
			}
			// done
			success = true;
		}
  	}

  	// do we need to update the max resolution used?
  	if (skipGlobalBitResolutionCalculation == false)
  	{
    	_bitResolution = newResolution;
    	if (_devices > 1)
    	{
      		for (uint8_t i = 0; i < _devices; i++)
      		{
        		if (_bitResolution == 12) break;
        		
        		DeviceAddress deviceAddr;
        		getAddress(deviceAddr, i);
        		uint8_t b = getResolution(deviceAddr);
        		if (b > _bitResolution) _bitResolution = b;
      		}
    	}
  	}

  	return success;
}

void Ecods18b20::writeScratchPad(const uint8_t* deviceAddress, const uint8_t* scratchPad)
{
	_wire->reset();
	_wire->select(deviceAddress);
	_wire->write(WRITESCRATCH);
	_wire->write(scratchPad[HIGH_ALARM_TEMP]); // high alarm temp
	_wire->write(scratchPad[LOW_ALARM_TEMP]); // low alarm temp

	// DS1820 and DS18S20 have no configuration register
	if (deviceAddress[DSROM_FAMILY] != DS18S20MODEL)
		_wire->write(scratchPad[CONFIGURATION]);

  	if (_autoSaveScratchPad)
    	saveScratchPad(deviceAddress);
  	else
    	_wire->reset();
}

// Sends command to one or more devices to save values from scratchpad to EEPROM
// If optional argument deviceAddress is omitted the command is send to all devices
// Returns true if no errors were encountered, false indicates failure
bool Ecods18b20::saveScratchPad(const uint8_t* deviceAddress)
{
	if (_wire->reset() == 0)
    	return false;

    if (deviceAddress == nullptr)
    	_wire->skip();
    else
    	_wire->select(deviceAddress);
  
  	_wire->write(COPYSCRATCH,_parasite);

 	// Specification: NV Write Cycle Time is typically 2ms, max 10ms
 	// Waiting 20ms to allow for sensors that take longer in practice
 	if (!_parasite)
 	{
    	delay(20);
    }
    else
    {
    	_activateExternalPullup();
    	delay(20);
    	_deactivateExternalPullup();
  	}
  
  	return _wire->reset() == 1;
}

void Ecods18b20::_activateExternalPullup() {
	if(_useExternalPullup)
		digitalWrite(pullupPin, LOW); // pullup is private. Check if we can rename with _pullupPin
}

void Ecods18b20::_deactivateExternalPullup() {
	if(_useExternalPullup)
		digitalWrite(pullupPin, HIGH);
}

// sends command for all devices on the bus to perform a temperature conversion
void Ecods18b20::requestTemperatures()
{
	_wire->reset();
	_wire->skip();
	_wire->write(STARTCONVO, _parasite);

	// ASYNC mode?
	if (!_waitForConversion)
		return;
	
	blockTillConversionComplete(_bitResolution);

}

// sends command for one device to perform a temperature by address
// returns FALSE if device is disconnected
// returns TRUE  otherwise
bool Ecods18b20::requestTemperaturesByAddress(const uint8_t* deviceAddress)
{
	uint8_t g_bitResolution = getResolution(deviceAddress);
	if (g_bitResolution == 0) {
		return false; //Device disconnected
	}

	_wire->reset();
	_wire->select(deviceAddress);
	_wire->write(STARTCONVO, _parasite);

	// ASYNC mode?
	if (!_waitForConversion)
		return true;

	blockTillConversionComplete(g_bitResolution);

	return true;

}

// sends command for one device to perform a temp conversion by index
bool Ecods18b20::requestTemperaturesByIndex(uint8_t deviceIndex)
{

	DeviceAddress deviceAddress;
	getAddress(deviceAddress, deviceIndex);

	return requestTemperaturesByAddress(deviceAddress);

}

// Fetch temperature for device index
float Ecods18b20::getTempCByIndex(uint8_t deviceIndex) {

	DeviceAddress deviceAddress;
	if (!getAddress(deviceAddress, deviceIndex)) {
		return DEVICE_DISCONNECTED_C;
	}
	return getTempC((uint8_t*) deviceAddress);
}
/*
// Fetch temperature for device index
float Ecods18b20::getTempFByIndex(uint8_t deviceIndex) {

	DeviceAddress deviceAddress;

	if (!getAddress(deviceAddress, deviceIndex)) {
		return DEVICE_DISCONNECTED_F;
	}

	return getTempF((uint8_t*) deviceAddress);

}
*/

// returns temperature in degrees C or DEVICE_DISCONNECTED_C if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_C is defined in
// DallasTemperature.h. It is a large negative number outside the
// operating range of the device
float Ecods18b20::getTempC(const uint8_t* deviceAddress) {
	return rawToCelsius(getTemp(deviceAddress));
}

// returns temperature in degrees F or DEVICE_DISCONNECTED_F if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_F is defined in
// DallasTemperature.h. It is a large negative number outside the
// operating range of the device
float Ecods18b20::getTempF(const uint8_t* deviceAddress) {
	return rawToFahrenheit(getTemp(deviceAddress));
}

// Continue to check if the IC has responded with a temperature
void Ecods18b20::blockTillConversionComplete(uint8_t bit_Resolution)
{

  if (_checkForConversion && !_parasite) {
    unsigned long start = millis();
    while (!isConversionComplete() && (millis() - start < MAX_CONVERSION_TIMEOUT ))
      yield();
  } else {
    unsigned long delms = millisToWaitForConversion(bit_Resolution);
    _activateExternalPullup();
    delay(delms);
    _deactivateExternalPullup();
  }

}

// returns temperature in 1/128 degrees C or DEVICE_DISCONNECTED_RAW if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_RAW is defined in
// DallasTemperature.h. It is a large negative number outside the
// operating range of the device
int16_t Ecods18b20::getTemp(const uint8_t* deviceAddress) {
	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad))
		return _calculateTemperature(deviceAddress, scratchPad);
	return DEVICE_DISCONNECTED_RAW;

}

// convert from raw to Celsius
float Ecods18b20::rawToCelsius(int16_t raw) {

	if (raw <= DEVICE_DISCONNECTED_RAW)
		return DEVICE_DISCONNECTED_C;
	// C = RAW/128
	return (float) raw * 0.0078125f;

}

// convert from raw to Fahrenheit
float Ecods18b20::rawToFahrenheit(int16_t raw) {

	if (raw <= DEVICE_DISCONNECTED_RAW)
		return DEVICE_DISCONNECTED_F;
	// C = RAW/128
	// F = (C*1.8)+32 = (RAW/128*1.8)+32 = (RAW*0.0140625)+32
	return ((float) raw * 0.0140625f) + 32.0f;

}

bool Ecods18b20::isConversionComplete() {
	uint8_t b = _wire->read_bit();
	return (b == 1);
}

// returns number of milliseconds to wait till conversion is complete (based on IC datasheet)
uint16_t Ecods18b20::millisToWaitForConversion(uint8_t bitResolution)
{
	switch (bitResolution) {
		case 9:
			return 94;
		case 10:
			return 188;
		case 11:
			return 375;
		default:
			return 750;
	}

}

// reads scratchpad and returns fixed-point temperature, scaling factor 2^-7
int16_t Ecods18b20::_calculateTemperature(const uint8_t* deviceAddress,
		uint8_t* scratchPad) {

	int16_t fpTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 11)
			| (((int16_t) scratchPad[TEMP_LSB]) << 3);

	/*
	 DS1820 and DS18S20 have a 9-bit temperature register.
	 Resolutions greater than 9-bit can be calculated using the data from
	 the temperature, and COUNT REMAIN and COUNT PER °C registers in the
	 scratchpad.  The resolution of the calculation depends on the model.
	 While the COUNT PER °C register is hard-wired to 16 (10h) in a
	 DS18S20, it changes with temperature in DS1820.
	 After reading the scratchpad, the TEMP_READ value is obtained by
	 truncating the 0.5°C bit (bit 0) from the temperature data. The
	 extended resolution temperature can then be calculated using the
	 following equation:
	                                  COUNT_PER_C - COUNT_REMAIN
	 TEMPERATURE = TEMP_READ - 0.25 + --------------------------
	                                         COUNT_PER_C
	 Hagai Shatz simplified this to integer arithmetic for a 12 bits
	 value for a DS18S20, and James Cameron added legacy DS1820 support.
	 See - http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html
	 */

	if ((deviceAddress[DSROM_FAMILY] == DS18S20MODEL) && (scratchPad[COUNT_PER_C] != 0))
	{
		fpTemperature = ((fpTemperature & 0xfff0) << 3) - 32
				+ (((scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN]) << 7)
						/ scratchPad[COUNT_PER_C]);
	}

	return fpTemperature;
}