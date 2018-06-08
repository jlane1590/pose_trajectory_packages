/*
 * PolstroSerialInterface.cpp
 *
 *  Created on: 10.10.2013
 *      Author: tuuzdu
 */

#include <vector>
#include <iostream>
#include <assert.h>
#include <polstro/PolstroSerialInterface.h>
#include <polstro/PolstroSerialInterfacePOSIX.h>

namespace Polstro {

SerialInterface::SerialInterface(const std::string& portName) {
}

SerialInterface::~SerialInterface() {
}

bool SerialInterface::setTarget(unsigned char channelNumber,
		unsigned short target) {
	if (!isOpen())
		return false;
	//assert((target == 0) || (target>=getMinChannelValue() && target<=getMaxChannelValue()));
	unsigned char command[4] = { 0x84, channelNumber, target & 0x7F, (target
			>> 7) & 0x7F };
	if (!writeBytes(command, sizeof(command)))
		return false;
	return true;
}

bool SerialInterface::setTargetMSS(unsigned char miniSCCChannelNumber,
		unsigned char normalizedTarget) {
	if (!isOpen())
		return false;
	assert( normalizedTarget>=0 && normalizedTarget<=254);
	unsigned char command[3] = { 0xFF, miniSCCChannelNumber, normalizedTarget };
	if (!writeBytes(command, sizeof(command)))
		return false;
	return true;
}

bool SerialInterface::setMultipleTargets(unsigned char numChannels, unsigned char firstChannel,
		unsigned short *targets) {
	if (!isOpen())
		return false;
  static const unsigned char initCmd[] = {0x9F, numChannels, firstChannel};
	std::vector<unsigned char> command (initCmd, initCmd + (sizeof(initCmd)/sizeof(initCmd[0])));
	unsigned short tar[2];
	for(int i = 0; i < numChannels; ++i){
		//assert((targets[i] == 0) || (targets[i]>=getMinChannelValue() && targets[i]<=getMaxChannelValue()));
		tar[0] = targets[i] & 0x7F;
		tar[1] = (targets[i] >> 7) & 0x7F;
		command.insert(command.end(), tar, tar+(sizeof(tar)/sizeof(tar[0])));
	}
	if (!writeBytes(&command[0], command.size()))
		return false;
	return true;
}

bool SerialInterface::setMultiplePos(unsigned char numChannels, unsigned char firstChannel, double *positions, bool crc_en) {
	if (!isOpen())
		return false;
  static const unsigned char initCmd[] = {0x9F, numChannels, firstChannel};
//	static const unsigned char initCmd[] = {0x83,0x01};
	std::vector<unsigned char> command (initCmd, initCmd + (sizeof(initCmd)/sizeof(initCmd[0])));
	unsigned short tar[2];
	for(int i = 0; i < numChannels; ++i){
    unsigned short target = degToPulse(positions[i]);
		assert((target == 0) || (target>=getMinChannelValue() && target<=getMaxChannelValue()));
		//assert((target == 0) || (target>=2500 && target<=getMaxChannelValue()));
    tar[0] = target & 0x7F;
		tar[1] = (target >> 7) & 0x7F;
		command.insert(command.end(), tar, tar+(sizeof(tar)/sizeof(tar[0])));
	}

	if(crc_en){
		command.push_back(0x00);
		command.back() = getCRC(&command[0], command.size() - 1);
	}
	
/*	for(std::vector<unsigned char>::const_iterator i = command.begin(); i != command.end(); ++i)
		std::cout << (int)*i << ' ';
	std::cout << std::endl;
*/

	if (!writeBytes(&command[0], command.size()))
		return false;
	return true;
}

bool SerialInterface::setSpeed(unsigned char channelNumber,
		unsigned short speed) {
	if (!isOpen())
		return false;
	unsigned char command[4] = { 0x87, channelNumber, speed & 0x7F, (speed >> 7)
			& 0x7F };
	if (!writeBytes(command, sizeof(command)))
		return false;
	return true;
}

bool SerialInterface::setAcceleration(unsigned char channelNumber,
		unsigned char acceleration) {
	if (!isOpen())
		return false;
	unsigned short accelerationAsShort = acceleration;
	unsigned char command[4] = { 0x89, channelNumber, accelerationAsShort
			& 0x7F, (accelerationAsShort >> 7) & 0x7F };
	if (!writeBytes(command, sizeof(command)))
		return false;
	return true;
}

bool SerialInterface::getPosition(unsigned char channelNumber,
		unsigned short& position) {
	if (!isOpen())
		return false;

	position = 0;

	unsigned char command[2] = { 0x90, channelNumber };
	if (!writeBytes(command, sizeof(command)))
		return false;

	unsigned char response[2] = { 0x00, 0x00 };
	if (!readBytes(response, sizeof(response)))
		return false;

	position = response[0] + 256 * response[1];
	return true;
}


bool SerialInterface::getMovingState(bool& servosAreMoving) {
	if (!isOpen())
		return false;

	servosAreMoving = false;
	unsigned char command = 0x93;
	if (!writeBytes(&command, sizeof(command)))
		return false;

	unsigned char response = 0x00;
	if (!readBytes(&response, sizeof(response)))
		return false;

	if (response != 0x00 && response != 0x01)
		return false;

	servosAreMoving = (response == 0x01);
	return true;
}


bool SerialInterface::getErrors(unsigned short& errors) {
	if (!isOpen())
		return false;

	unsigned char command = 0xA1;
	if (!writeBytes(&command, sizeof(command)))
		return false;

	unsigned char response[2] = { 0x00, 0x00 };
	if (!readBytes(response, sizeof(response)))
		return false;

	errors = (response[0] & 0x7F) + 256 * (response[1] & 0x7F); // Need to check this code on real errors!
	return true;
}


bool SerialInterface::goHome() {
	if (!isOpen())
		return false;

	unsigned char command = 0xA2;
	if (!writeBytes(&command, sizeof(command)))
		return false;
	return true;
}

unsigned short SerialInterface::degToPulse(double posD)
{
  double ratio = (mMaxChannelValue - mMinChannelValue)/servoRange;
  return (baseValue - (posD - homePos)*ratio);
}

unsigned char SerialInterface::getCRC(unsigned char *message, unsigned char length)
{
  unsigned char i, j, crc = 0;
 
  for (i = 0; i < length; i++)
  {
    crc ^= message[i];
    for (j = 0; j < 8; j++)
    {
      if (crc & 1)
        crc ^= CRC7_POLY;
      crc >>= 1;
    }
  }
  return crc;
}


SerialInterface* SerialInterface::createSerialInterface(
		const std::string& portName, unsigned int baudRate) {
	SerialInterface* serialInterface = NULL;

	//printf("Creating serial interface '%s'\n", portName.c_str());
	serialInterface = new SerialInterfacePOSIX(portName);

	return serialInterface;
}

}
;
