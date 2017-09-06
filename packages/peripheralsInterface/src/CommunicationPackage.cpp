 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * CommunicationPackage.cpp
 *
 *  Created on: Jun 7, 2016
 *      Author: Edwin Schreuder
 */

#include <iostream>
#include <mutex>
#include <cstring>

#include "int/CommunicationPackage.hpp"

using namespace std;

CommunicationPackage::CommunicationPackage(size_t payloadIndex) :
		payloadIndex(payloadIndex) {
	payloadLength = 0;
}

CommunicationPackage::~CommunicationPackage() {
}

unsigned char * CommunicationPackage::getBuffer() {
	return buffer;
}

size_t CommunicationPackage::getLength() {
	return payloadIndex + payloadLength;
}

void CommunicationPackage::print() {
	for (size_t i = 0; i < getLength(); i++) {
		cout << "0x" << hex << ((int) getBuffer()[i]) << dec << " ";
	}
	cout << endl;
}

unsigned char CommunicationPackage::getAckId() {
	return buffer[1];
}

unsigned char CommunicationPackage::getPayloadSize() {
	return buffer[payloadIndex - 1];
}

unsigned char CommunicationPackage::getCheckSum() {
	return buffer[payloadIndex - 2];
}

template <typename T> T CommunicationPackage::getData(size_t position) {
	T * value = (T *) &(buffer[payloadIndex + position * sizeof(T)]);
	return *value;
}

template uint8_t CommunicationPackage::getData<uint8_t>(size_t position);
template uint16_t CommunicationPackage::getData<uint16_t>(size_t position);
template uint32_t CommunicationPackage::getData<uint32_t>(size_t position);
template uint64_t CommunicationPackage::getData<uint64_t>(size_t position);
template int8_t CommunicationPackage::getData<int8_t>(size_t position);
template int16_t CommunicationPackage::getData<int16_t>(size_t position);
template int32_t CommunicationPackage::getData<int32_t>(size_t position);
template int64_t CommunicationPackage::getData<int64_t>(size_t position);

unsigned char CommunicationPackage::calculateCheckSum() {
	unsigned char checkSum = 0;

	for (size_t i = 0; i < getLength(); i++) {
		checkSum += buffer[i];
	}

	return checkSum - getCheckSum();
}


ReceivePackage::ReceivePackage(unsigned char * package, size_t size) :
		CommunicationPackage(7) {
	memcpy(buffer, package, size);
	payloadLength = size - payloadIndex;
}

ResponseList ReceivePackage::getResponseType() {
	return (ResponseList) buffer[4];
}

unsigned char ReceivePackage::getBufferSpace() {
	return buffer[3];
}

unsigned char ReceivePackage::getFeedbackId() {
	return buffer[2];
}

bool ReceivePackage::isPacketValid() {
	bool packetValid = true;

	if (getCheckSum() != calculateCheckSum()) {
		packetValid = false;
	}

	if ((unsigned char) getPayloadSize() - payloadLength) {
		packetValid = false;
	}

	return packetValid;
}

TransmitPackage::TransmitPackage(CommandList command) :
	CommunicationPackage(5){
	buffer[2] = (unsigned char) command;
}

unsigned char * TransmitPackage::getBuffer() {
	buffer[0] = SOF;
	setPayloadSize(payloadLength);
	setCheckSum(0);
	setCheckSum(calculateCheckSum());
	return buffer;
}


void TransmitPackage::setAckId(unsigned char ackId) {
	buffer[1] = ackId;
}

CommandList TransmitPackage::getCommand() {
	return (CommandList) buffer[2];
}

void TransmitPackage::setPayloadSize(unsigned char payloadSize) {
	buffer[payloadIndex - 1] = payloadSize;
}

void TransmitPackage::setCheckSum(unsigned char checkSum) {
	buffer[payloadIndex - 2] = checkSum;
}

template <typename T> void TransmitPackage::addData(T data) {
	T * value = (T *) &(buffer[payloadIndex + payloadLength]);
	*value = data;
	payloadLength += sizeof(T);
}

template void TransmitPackage::addData<uint8_t>(uint8_t data);
template void TransmitPackage::addData<uint16_t>(uint16_t data);
template void TransmitPackage::addData<uint32_t>(uint32_t data);
template void TransmitPackage::addData<uint64_t>(uint64_t data);
template void TransmitPackage::addData<int8_t>(int8_t data);
template void TransmitPackage::addData<int16_t>(int16_t data);
template void TransmitPackage::addData<int32_t>(int32_t data);
template void TransmitPackage::addData<int64_t>(int64_t data);

