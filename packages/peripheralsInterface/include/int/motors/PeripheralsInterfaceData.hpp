 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 
#ifndef INCLUDE_CPERIPHERALSINTERFACEDATA_HPP_
#define INCLUDE_CPERIPHERALSINTERFACEDATA_HPP_

#include <atomic>
#include <mutex>

#include "int/PeripheralsInterfaceTypes.hpp"

#include "falconsCommon.hpp"


using namespace std;

class PeripheralsInterfaceMotionBoardData {

public:
	MotionBoardDataOutput getDataOutput();
	void setDataOutput(const MotionBoardDataOutput& dataOutput);

	MotionBoardSettings getSettings();
	void setSettings(const MotionBoardSettings& settings);
	bool isSettingsChanged();

	bool isOnline();
	void setOnline(bool online);

private:
	// Motion board input and output data
	MotionBoardDataOutput _dataOutput;
	mutex _dataOutputLock;

	MotionBoardSettings _settings;
	bool _settingsChanged;
	mutex _settingsLock;

	atomic<bool> _online;
};

class PeripheralsInterfaceBallhandlerBoardData {

public:
	BallhandlerBoardDataOutput getDataOutput();
	void setDataOutput(const BallhandlerBoardDataOutput& dataOutput);

	BallhandlerBoardSettings getSettings();
	void setSettings(const BallhandlerBoardSettings& settings);
	bool isSettingsChanged();

	bool isOnline();
	void setOnline(bool online);

private:
	// Ballhandler board input and output data
	BallhandlerBoardDataOutput _dataOutput;
	mutex _dataOutputLock;

	BallhandlerBoardSettings _settings;
	bool _settingsChanged;
	mutex _settingsLock;

	atomic<bool> _online;
};

class PeripheralsInterfaceData {
public:
	PeripheralsInterfaceData();
	~PeripheralsInterfaceData();

	// Motion board data
	PeripheralsInterfaceMotionBoardData& getLeftMotionBoard();
	PeripheralsInterfaceMotionBoardData& getRightMotionBoard();
	PeripheralsInterfaceMotionBoardData& getRearMotionBoard();

	// Ballhandler board data
	PeripheralsInterfaceBallhandlerBoardData& getLeftBallhandlerBoard();
	PeripheralsInterfaceBallhandlerBoardData& getRightBallhandlerBoard();

	// Motion adapter data
	piVelAcc getVelocityInput();
	void setVelocityInput(const piVelAcc& pos);
	bool isVelocityInputChanged();

	piVelAcc getVelocityOutput();
	void setVelocityOutput(const piVelAcc& pos);

	piDisplacement getDisplacementOutput();
	void setDisplacementOutput(const piDisplacement& displacement);

	// Robot is active with in/out of play
	bool isRobotActive();
	void setRobotActive(bool robotActive);

	// Ballhandler settings
	BallhandlerSettings getBallhandlerSettings();
	void setBallhandlerSettings(const BallhandlerSettings& settings);

    // Ballhandler input and output
    BallhandlerFeedback getBallhandlerFeedback();
    BallhandlerSetpoints getBallhandlerSetpoints();
    void setBallhandlerFeedback(BallhandlerFeedback const &feedback);
    void setBallhandlerSetpoints(BallhandlerSetpoints const &setpoints);
    
	// Number of connected devices
	size_t getNumberOfConnectedDevices();
	void setNumberOfConnectedDevices(size_t numberOfConnectedDevices);

private:
	PeripheralsInterfaceMotionBoardData _motionBoardLeft;
	PeripheralsInterfaceMotionBoardData _motionBoardRight;
	PeripheralsInterfaceMotionBoardData _motionBoardRear;

	PeripheralsInterfaceBallhandlerBoardData _ballhandlerBoardLeft;
	PeripheralsInterfaceBallhandlerBoardData _ballhandlerBoardRight;

	// Velocity input
	piVelAcc _velInput;
	bool _velInputChanged;
	mutex _velInputLock;

	// Velocity output
	piVelAcc _velOutput;
	mutex _velOutputLock;

	// Displacement output
	piDisplacement _displacementOutput;
	mutex _displacementOutputLock;

	// Data whether robot is active or not
	atomic<bool> _robotActive;

	// Motion settings
	MotionSettings _motionSettings;
	bool _motionSettingsChanged;
	mutex _motionSettingsLock;

	// Ballhandler Settings
	BallhandlerSettings _ballhandlerSettings;
	mutex _ballhandlerSettingsLock;
	
    // Ballhandler input and output
    BallhandlerSetpoints _ballhandlerSetpoints;
    mutex _ballhandlerSetpointsLock;

    BallhandlerFeedback _ballhandlerFeedback;
    mutex _ballhandlerFeedbackLock;

	// Number of connected devices
	atomic<size_t> _numberOfConnectedDevices;
};

#endif /* INCLUDE_CPERIPHERALSINTERFACEDATA_HPP_ */
