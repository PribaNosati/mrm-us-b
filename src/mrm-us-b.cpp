#include "mrm-us-b.h"
#include <mrm-robot.h>

/** Constructor
@param robot - robot containing this board
@param esp32CANBusSingleton - a single instance of CAN Bus common library for all CAN Bus peripherals.
@param hardwareSerial - Serial, Serial1, Serial2,... - an optional serial port, for example for Bluetooth communication
@param maxNumberOfBoards - maximum number of boards
*/
Mrm_us_b::Mrm_us_b(uint8_t maxNumberOfBoards) : 
	SensorBoard(1, "US-B", maxNumberOfBoards, ID_MRM_US_B, 1) {
	readings = new std::vector<uint16_t>(maxNumberOfBoards);
}

Mrm_us_b::~Mrm_us_b()
{
}

/** Add a mrm-us-b sensor
@param deviceName - device's name
*/
void Mrm_us_b::add(char * deviceName)
{
	uint16_t canIn, canOut;
	switch (nextFree) {
	case 0:
		canIn = CAN_ID_US_B0_IN;
		canOut = CAN_ID_US_B0_OUT;
		break;
	case 1:
		canIn = CAN_ID_US_B1_IN;
		canOut = CAN_ID_US_B1_OUT;
		break;
	case 2:
		canIn = CAN_ID_US_B2_IN;
		canOut = CAN_ID_US_B2_OUT;
		break;
	case 3:
		canIn = CAN_ID_US_B3_IN;
		canOut = CAN_ID_US_B3_OUT;
		break;
	case 4:
		canIn = CAN_ID_US_B4_IN;
		canOut = CAN_ID_US_B4_OUT;
		break;
	case 5:
		canIn = CAN_ID_US_B5_IN;
		canOut = CAN_ID_US_B5_OUT;
		break;
	case 6:
		canIn = CAN_ID_US_B6_IN;
		canOut = CAN_ID_US_B6_OUT;
		break;
	case 7:
		canIn = CAN_ID_US_B7_IN;
		canOut = CAN_ID_US_B7_OUT;
		break;
	default:
		sprintf(errorMessage, "Too many %s: %i.", _boardsName.c_str(), nextFree);
		return;
	}

	SensorBoard::add(deviceName, canIn, canOut);
}

/** Read CAN Bus message into local variables
@param data - 8 bytes from CAN Bus message.
@param length - number of data bytes
*/
bool Mrm_us_b::messageDecode(CANMessage& message) {
	for (Device& device : devices)
		if (isForMe(message.id, device)) {
			if (!messageDecodeCommon(message, device)) {
				switch (message.data[0]) {
					case COMMAND_SENSORS_MEASURE_SENDING:
					{
						uint16_t mm = (message.data[2] << 8) | message.data[1];
						(*readings)[device.number] = mm;
						device.lastReadingsMs = millis();
					}
					break;
				// }
				default:
					errorAdd(message, ERROR_COMMAND_UNKNOWN, false, true);
				}
			}
			return true;
		}
	return false;
}

/** Analog readings
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - analog value
*/
uint16_t Mrm_us_b::reading(uint8_t deviceNumber) {
	if (deviceNumber >= nextFree) {
		sprintf(errorMessage, "%s %i doesn't exist.", _boardsName.c_str(), deviceNumber);
		return 0;
	}
	aliveWithOptionalScan(&devices[deviceNumber], true);
	if (started(deviceNumber))
		return (*readings)[deviceNumber];
	else
		return 0;
}

/** Print all readings in a line
*/
void Mrm_us_b::readingsPrint() {
	print("US:");
	for (Device& device: devices)
		print(" %3i", (*readings)[device.number]);
}

/** If sensor not started, start it and wait for 1. message
@param deviceNumber - Device's ordinal number. Each call of function add() assigns a increasing number to the device, starting with 0.
@return - started or not
*/
bool Mrm_us_b::started(uint8_t deviceNumber) {
	if (millis() - devices[deviceNumber].lastReadingsMs > MRM_US_B_INACTIVITY_ALLOWED_MS || devices[deviceNumber].lastReadingsMs == 0) {
		//print("Start mrm-us-b%i \n\r", deviceNumber); 
		for (uint8_t i = 0; i < 8; i++) { // 8 tries
			start(&devices[deviceNumber], 0);
			// Wait for 1. message.
			uint32_t startMs = millis();
			while (millis() - startMs < 50) {
				if (millis() - devices[deviceNumber].lastReadingsMs < 100) {
					//print("US confirmed\n\r");
					return true;
				}
				delayMs(1);
			}
		}
		strcpy(errorMessage, "mrm-us-b dead.\n\r");
		return false;
	}
	else
		return true;
}

/**Test
*/
void Mrm_us_b::test()
{
	static uint64_t lastMs = 0;

	if (millis() - lastMs > 300) {
		uint8_t pass = 0;
		for (Device& device: devices){
			if (device.alive) {
				if (pass++)
					print("| ");
				print("%i ", reading(device.number));
			}
		}
		lastMs = millis();
		if (pass)
			print("\n\r");
	}
}
