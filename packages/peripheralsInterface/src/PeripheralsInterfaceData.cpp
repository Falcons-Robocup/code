 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include <cDiagnosticsEvents.hpp>
#include <int/PeripheralsInterfaceData.hpp>

using namespace std;

PeripheralsInterfaceData::PeripheralsInterfaceData() {
	_velInputChanged = false;
	_hasBall = false;
	_compassValue = 0.0;
	_robotActive = false;
	_ballhandlerAngle = 0.0;
	_motionSettingsChanged = false;
	_ballhandlerSettingsChanged = false;
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
	_displacementOutput.pos = geometry::Pose2D();
	_displacementOutputLock.unlock();

	return displacement;
}

void PeripheralsInterfaceData::setDisplacementOutput(const piDisplacement& displacement) {
	_displacementOutputLock.lock();
	_displacementOutput.pos.shift(
			displacement.pos.getX(),
			displacement.pos.getY());
	_displacementOutput.pos.turn(displacement.pos.getPhi());
	_displacementOutputLock.unlock();
}

passBall PeripheralsInterfaceData::getPassBallParams() {
	_passBallParamsLock.lock();
	passBall passBallParams = _passBallParams;
	_passBallParamsLock.unlock();

	return passBallParams;
}

void PeripheralsInterfaceData::setPassBallParams(const passBall &passBallParams) {
	_passBallParamsLock.lock();
	_passBallParams = passBallParams;
	_passBallParamsLock.unlock();
}

bool PeripheralsInterfaceData::getHasBall() {
	return _hasBall;
}

void PeripheralsInterfaceData::setHasBall(bool hasBall) {
	_hasBall = hasBall;
}

bool PeripheralsInterfaceData::isRobotActive() {
	return _robotActive;
}

void PeripheralsInterfaceData::setRobotActive(bool active) {
	_robotActive = active;
}

float PeripheralsInterfaceData::getBallhandlerAngle() {
	return _ballhandlerAngle;
}

void PeripheralsInterfaceData::setBallhandlerAngle(float angle) {
	_ballhandlerAngle = angle;
}

MotionSettings PeripheralsInterfaceData::getMotionSettings() {
	_motionSettingsLock.lock();
	MotionSettings settings = _motionSettings;
	_motionSettingsLock.unlock();

	return settings;
}

void PeripheralsInterfaceData::setMotionSettings(const MotionSettings& motionsettings) {
	_motionSettingsLock.lock();
	_motionSettings = motionsettings;
	_motionSettingsChanged = true;
	_motionSettingsLock.unlock();
}

bool PeripheralsInterfaceData::isMotionSettingsChanged() {
	_motionSettingsLock.lock();
	bool motionSettingsChanged = _motionSettingsChanged;
	_motionSettingsChanged = false;
	_motionSettingsLock.unlock();

	return motionSettingsChanged;
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
	_ballhandlerSettingsChanged = true;
	_ballhandlerSettingsLock.unlock();
}

bool PeripheralsInterfaceData::isBallhandlerSettingsChanged() {
	_ballhandlerSettingsLock.lock();
	bool ballhandlerSettingsChanged = _ballhandlerSettingsChanged;
	_ballhandlerSettingsChanged = false;
	_ballhandlerSettingsLock.unlock();

	return ballhandlerSettingsChanged;
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
