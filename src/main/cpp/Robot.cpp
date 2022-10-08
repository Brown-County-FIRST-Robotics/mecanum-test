#include <iostream>
#include <string>

#include "frc/TimedRobot.h"
#include "frc/Joystick.h"
#include "frc/Solenoid.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "ctre/Phoenix.h"
#include "DrivebaseSimFX.h"
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc/drive/MecanumDrive.h"

using namespace frc;

class Robot : public TimedRobot
{
public:
	WPI_TalonFX back_left{0};
	WPI_TalonFX front_left{1};
	WPI_TalonFX front_right{2};
	WPI_TalonFX back_right{3};

	WPI_TalonFX shooter_top{6};
	WPI_TalonFX shooter_bottom{7};
	WPI_TalonSRX shooter_angle_1{11};
	WPI_TalonSRX shooter_angle_2{10};
	WPI_PigeonIMU _pigeon{0};
	WPI_TalonFX intake{12};
	// Solenoid shooter_solenoid;

	Joystick _joystick{0};

	// DifferentialDrive _diffDrive{front_left, front_right};
	MecanumDrive mecDrive{front_left, back_left, front_right, back_right};

	void SimulationPeriodic()
	{
		_driveSim.Run();
	}

	void TeleopPeriodic()
	{
		/*
		0:left x
		1:left Y
		2:right x
		3:right y

		*/

		double joyX = _joystick.GetRawAxis(0);
		double joyY = -_joystick.GetRawAxis(1);
		double joyR = _joystick.GetRawAxis(2);
		/* deadband gamepad 5%*/
		if (fabs(joyR) < 0.05)
			joyR = 0;
		if (fabs(joyX) < 0.05)
			joyX = 0;
		if (fabs(joyY) < 0.05)
			joyY = 0;
		/* drive robot */
		mecDrive.DriveCartesian(joyY, joyX, joyR);
	}

	void RobotInit()
	{
		/* factory default values */
		front_right.ConfigFactoryDefault();
		back_right.ConfigFactoryDefault();
		front_left.ConfigFactoryDefault();
		back_left.ConfigFactoryDefault();

		front_right.SetInverted(TalonFXInvertType::Clockwise);
		back_right.SetInverted(TalonFXInvertType::FollowMaster);
		front_left.SetInverted(TalonFXInvertType::CounterClockwise);
		back_left.SetInverted(TalonFXInvertType::FollowMaster);

		frc::SmartDashboard::PutData("Field", &_driveSim.GetField());
	}

private:
	DrivebaseSimFX _driveSim{front_left, front_right, _pigeon};
};

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif