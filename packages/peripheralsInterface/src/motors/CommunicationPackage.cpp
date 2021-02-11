// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CommunicationPackage.cpp
 *
 *  Created on: Jun 7, 2016
 *      Author: Edwin Schreuder
 */

#include <iostream>
#include <mutex>
#include <cstring>

#include "int/motors/CommunicationPackage.hpp"

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

