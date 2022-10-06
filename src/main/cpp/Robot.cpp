/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
/**
 * Enable robot and slowly drive forward.
 * [1] If DS reports errors, adjust CAN IDs and firmware update.
 * [2] If motors are spinning incorrectly, first check gamepad.
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 */
#include <iostream>
#include <string>

#include "frc/TimedRobot.h"
#include "frc/Joystick.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/drive/DifferentialDrive.h"
#include "ctre/Phoenix.h"
#include "DrivebaseSimFX.h"


using namespace frc;

class Robot: public TimedRobot {
public:
	/* ------ [1] Update CAN Device IDs and switch to WPI_VictorSPX where necessary ------*/
	WPI_TalonFX front_right{1};
	WPI_TalonFX back_right{3};
	WPI_TalonFX front_left{2};
	WPI_TalonFX back_left{4};
	WPI_TalonFX shooter_top{5};
	WPI_TalonFX shooter_bottom{6};
	WPI_TalonSRX shooter_angle_1{7};
	WPI_TalonSRX shooter_angle_2{8};
	WPI_PigeonIMU _pigeon{0};
	WPI_TalonFX intake{9};

	DifferentialDrive _diffDrive{front_left, front_right};

	Joystick _joystick{0};

	void SimulationPeriodic() {
		_driveSim.Run();
	}

	void TeleopPeriodic() {

		std::stringstream work;

		/* get gamepad stick values */
		double forw = -_joystick.GetRawAxis(1); /* positive is forward */
		double turn = _joystick.GetRawAxis(2); /* positive is right */

		/* deadband gamepad 10%*/
		if (fabs(forw) < 0.10)
			forw = 0;
		if (fabs(turn) < 0.10)
			turn = 0;

		/* drive robot */
		_diffDrive.ArcadeDrive(forw, turn, false);

		/* -------- [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for RIGHT */
		work << " GF:" << forw << " GT:" << turn;

		/* get sensor values */
		//double leftPos = front_left.GetSelectedSensorPosition(0);
		//double rghtPos = front_right.GetSelectedSensorPosition(0);
		double leftVelUnitsPer100ms = front_left.GetSelectedSensorVelocity(0);
		double rghtVelUnitsPer100ms = front_right.GetSelectedSensorVelocity(0);

		work << " L:" << leftVelUnitsPer100ms << " R:" << rghtVelUnitsPer100ms;

		/* print to console */
		std::cout << work.str() << std::endl;
	}

	void RobotInit() {
		/* factory default values */
		front_right.ConfigFactoryDefault();
		back_right.ConfigFactoryDefault();
		front_left.ConfigFactoryDefault();
		back_left.ConfigFactoryDefault();

		/* set up followers */
		back_right.Follow(front_right);
		back_left.Follow(front_left);

		/* [3] flip values so robot moves forward when stick-forward/LEDs-green */
		front_right.SetInverted(TalonFXInvertType::Clockwise);
		back_right.SetInverted(TalonFXInvertType::FollowMaster);
		front_left.SetInverted(TalonFXInvertType::CounterClockwise);
		back_left.SetInverted(TalonFXInvertType::FollowMaster);

		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
		// front_right.SetSensorPhase(true);
		// front_left.SetSensorPhase(true);

		frc::SmartDashboard::PutData("Field", &_driveSim.GetField());
	}

private:
	DrivebaseSimFX _driveSim{front_left, front_right, _pigeon};
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif