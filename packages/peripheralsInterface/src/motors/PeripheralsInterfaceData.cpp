// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <cDiagnostics.hpp>
#include "int/motors/PeripheralsInterfaceData.hpp"

using namespace std;

PeripheralsInterfaceData::PeripheralsInterfaceData() {
	_velInputChanged = false;
	_robotActive = false;
	_motionSettingsChanged = false;
}

PeripheralsInterfaceData::~PeripheralsInterfaceData() {

}

PeripheralsInterfaceMotionBoardData& PeripheralsInterfaceData::getLeftMotionBoard() {
	return _motionBoardLeft;
}

PeripheralsInterfaceMotionBoardData& PeripheralsInterfaceData::getRightMotionBoard() {
	return _motionBoardRight;
}

PeripheralsInterfaceMotionBoardData& PeripheralsInterfaceData::getRearMotionBoard() {
	return _motionBoardRear;
}

PeripheralsInterfaceBallhandlerBoardData& PeripheralsInterfaceData::getLeftBallhandlerBoard() {
	return _ballhandlerBoardLeft;
}

PeripheralsInterfaceBallhandlerBoardData& PeripheralsInterfaceData::getRightBallhandlerBoard() {
	return _ballhandlerBoardRight;
}

piVelAcc PeripheralsInterfaceData::getVelocityInput() {
	_velInputLock.lock();
	piVelAcc velocityInput = _velInput;
	_velInputLock.unlock();

	return velocityInput;
}

void PeripheralsInterfaceData::setVelocityInput(const piVelAcc& vel) {
	_velInputLock.lock();
	_velInput = vel;
	_velInputChanged = true;
	_velInputLock.unlock();
}

bool PeripheralsInterfaceData::isVelocityInputChanged() {
	_velInputLock.lock();
	bool velocityInputChanged = _velInputChanged;
	_velInputChanged = false;
	_velInputLock.unlock();

	return velocityInputChanged;
}

piVelAcc PeripheralsInterfaceData::getVelocityOutput() {
	_velOutputLock.lock();
	piVelAcc velocityOutput = _velOutput;
	_velOutputLock.unlock();

	return velocityOutput;
}

void PeripheralsInterfaceData::setVelocityOutput(const piVelAcc& vel) {
	_velOutputLock.lock();
	_velOutput = vel;
	_velOutputLock.unlock();
}

piDisplacement PeripheralsInterfaceData::getDisplacementOutput() {
	_displacementOutputLock.lock();
	piDisplacement displacement = _displacementOutput;
	_displacementOutputLock.unlock();

	return displacement;
}

void PeripheralsInterfaceData::setDisplacementOutput(const piDisplacement& displacement) {
	_displacementOutputLock.lock();
	_displacementOutput.m1_pos = displacement.m1_pos;
	_displacementOutput.m2_pos = displacement.m2_pos;
	_displacementOutput.m3_pos = displacement.m3_pos;
	_displacementOutputLock.unlock();
}

bool PeripheralsInterfaceData::isRobotActive() {
	return _robotActive;
}

void PeripheralsInterfaceData::setRobotActive(bool active) {
	_robotActive = active;
}

BallhandlerSettings PeripheralsInterfaceData::getBallhandlerSettings() {
	_ballhandlerSettingsLock.lock();
	BallhandlerSettings ballhandlerSettings = _ballhandlerSettings;
	_ballhandlerSettingsLock.unlock();

	return ballhandlerSettings;
}

void PeripheralsInterfaceData::setBallhandlerSettings(const BallhandlerSettings& settings) {
	_ballhandlerSettingsLock.lock();
	_ballhandlerSettings = settings;
	_ballhandlerSettingsLock.unlock();
}

BallhandlerSetpoints PeripheralsInterfaceData::getBallhandlerSetpoints()
{
    _ballhandlerSetpointsLock.lock();
    BallhandlerSetpoints result = _ballhandlerSetpoints;
    _ballhandlerSetpointsLock.unlock();
    return result;
}

void PeripheralsInterfaceData::setBallhandlerSetpoints(BallhandlerSetpoints const &setpoints)
{
    _ballhandlerSetpointsLock.lock();
    _ballhandlerSetpoints = setpoints;
    _ballhandlerSetpointsLock.unlock();
}

BallhandlerFeedback PeripheralsInterfaceData::getBallhandlerFeedback()
{
    _ballhandlerFeedbackLock.lock();
    BallhandlerFeedback result = _ballhandlerFeedback;
    _ballhandlerFeedbackLock.unlock();
    return result;
}

void PeripheralsInterfaceData::setBallhandlerFeedback(BallhandlerFeedback const &feedback)
{
	_ballhandlerFeedbackLock.lock();
    _ballhandlerFeedback = feedback;
    _ballhandlerFeedbackLock.unlock();
}

size_t PeripheralsInterfaceData::getNumberOfConnectedDevices() {
	return _numberOfConnectedDevices;
}

void PeripheralsInterfaceData::setNumberOfConnectedDevices(size_t numberOfConnectedDevices) {
	_numberOfConnectedDevices = numberOfConnectedDevices;
}

/*
 * MotionBoardData Class Methods
 */
MotionBoardDataOutput PeripheralsInterfaceMotionBoardData::getDataOutput() {
	_dataOutputLock.lock();
	MotionBoardDataOutput dataOutput = _dataOutput;
	_dataOutputLock.unlock();

	return dataOutput;
}

void PeripheralsInterfaceMotionBoardData::setDataOutput(const MotionBoardDataOutput& dataOutput) {
	_dataOutputLock.lock();
	_dataOutput = dataOutput;
	_dataOutputLock.unlock();
}

MotionBoardSettings PeripheralsInterfaceMotionBoardData::getSettings() {
	_settingsLock.lock();
	MotionBoardSettings settings = _settings;
	_settingsLock.unlock();

	return settings;
}

void PeripheralsInterfaceMotionBoardData::setSettings(const MotionBoardSettings& settings) {
	_settingsLock.lock();
	_settings = settings;
	_settingsChanged = true;
	_settingsLock.unlock();
}

bool PeripheralsInterfaceMotionBoardData::isSettingsChanged() {
	_settingsLock.lock();
	bool settingsChanged = _settingsChanged;
	_settingsChanged = false;
	_settingsLock.unlock();

	return settingsChanged;
}

bool PeripheralsInterfaceMotionBoardData::isOnline() {
	return _online;
}

void PeripheralsInterfaceMotionBoardData::setOnline(bool online) {
	_online = online;
}

/*
 * BallhandlerBoardData Class Methods
 */
BallhandlerBoardDataOutput PeripheralsInterfaceBallhandlerBoardData::getDataOutput() {
	_dataOutputLock.lock();
	BallhandlerBoardDataOutput dataOutput = _dataOutput;
	_dataOutputLock.unlock();

	return dataOutput;
}

void PeripheralsInterfaceBallhandlerBoardData::setDataOutput(const BallhandlerBoardDataOutput& dataOutput) {
	_dataOutputLock.lock();
	_dataOutput = dataOutput;
	_dataOutputLock.unlock();
}

BallhandlerBoardSettings PeripheralsInterfaceBallhandlerBoardData::getSettings() {
	_settingsLock.lock();
	BallhandlerBoardSettings settings = _settings;
	_settingsLock.unlock();

	return settings;
}

void PeripheralsInterfaceBallhandlerBoardData::setSettings(const BallhandlerBoardSettings& settings) {
	_settingsLock.lock();
	_settings = settings;
	_settingsChanged = true;
	_settingsLock.unlock();
}

bool PeripheralsInterfaceBallhandlerBoardData::isSettingsChanged() {
	_settingsLock.lock();
	bool settingsChanged = _settingsChanged;
	_settingsChanged = false;
	_settingsLock.unlock();

	return settingsChanged;
}

bool PeripheralsInterfaceBallhandlerBoardData::isOnline() {
	return _online;
}

void PeripheralsInterfaceBallhandlerBoardData::setOnline(bool online) {
	_online = online;
}
