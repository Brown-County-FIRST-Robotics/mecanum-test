/*
TODO: add rumble
TODO: add autonomous functions
*/
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
#include "frc/DoubleSolenoid.h"
#include "frc/Timer.h"
#include "frc/PneumaticsControlModule.h"
#include "frc/GenericHID.h"
#include "units/time.h"
#include "frc/XboxController.h"

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
	WPI_PigeonIMU pigeon{0};
	WPI_TalonFX intake{12};

	DoubleSolenoid shooter_solenoid{PneumaticsModuleType::CTREPCM, 0, 1};
	Timer shooter_solenoid_timer;
	int shooter_solenoid_previous_position = shooter_solenoid.kReverse;

	DoubleSolenoid intake_solenoid{PneumaticsModuleType::CTREPCM, 2, 3};
	Timer intake_solenoid_timer;

	MotorControllerGroup shooter_angle{shooter_angle_2, shooter_angle_1};

	XboxController controller{0};

	GenericHID hid{0};
	float driveSpeed = 0.5;

	MecanumDrive mecDrive{front_left, back_left, front_right, back_right};

	void SimulationPeriodic()
	{
		driveSim.Run();
	}

	void TeleopPeriodic()
	{
		double joyX = controller.GetLeftX();
		double joyY = controller.GetLeftY();
		double joyR = controller.GetRightX();

		/* deadband gamepad 5%*/
		if (fabs(joyR) < 0.05)
			joyR = 0;
		if (fabs(joyX) < 0.05)
			joyX = 0;
		if (fabs(joyY) < 0.05)
			joyY = 0;
		mecDrive.DriveCartesian(joyY, joyX, joyR);
		if (controller.GetPOV() == -1 || controller.GetPOV() == 90 || controller.GetPOV() == 270)
		{
			shooter_angle.Set(0);
		}
		else if (controller.GetPOV() == 0)
		{
			shooter_angle.Set(-.5);
		}
		else if (controller.GetPOV() == 180)
		{
			shooter_angle.Set(.5);
		}

		double rTrigger = controller.GetRightTriggerAxis();
		double lTrigger = controller.GetLeftTriggerAxis();

		if (rTrigger > lTrigger)
		{
			intake.Set(rTrigger);
			shooter_bottom.Set(rTrigger);
			shooter_top.Set(rTrigger);
		}
		if (rTrigger < lTrigger)
		{
			intake.Set(lTrigger * -.8);
			shooter_bottom.Set(lTrigger * -.8);
			shooter_top.Set(lTrigger * -.8);
		}

		if (rTrigger == lTrigger)
		{
			intake.Set(0);
			shooter_bottom.Set(0);
			shooter_top.Set(0);
		}

		/*if(rTrigger!=0){
			RUMBLE
		}else{
			DO NOT RUMBLE
		}*/

		// solenoids:
		if (controller.GetAButton())
		{
			shooter_solenoid_timer.Start();
			if (shooter_solenoid_previous_position == DoubleSolenoid::kForward)
			{
				shooter_solenoid.Set(DoubleSolenoid::kReverse);
				shooter_solenoid_previous_position = DoubleSolenoid::kReverse;
			}
			if (shooter_solenoid_previous_position == DoubleSolenoid::kReverse)
			{
				shooter_solenoid.Set(DoubleSolenoid::kForward);
				shooter_solenoid_previous_position = DoubleSolenoid::kForward;
			}
		}
		units::second_t quarterOfASecond = .25_s;
		if (shooter_solenoid_timer.HasElapsed(quarterOfASecond))
		{
			shooter_solenoid_timer.Stop();
			shooter_solenoid_timer.Reset();
			shooter_solenoid_previous_position = shooter_solenoid.Get();
			shooter_solenoid.Set(DoubleSolenoid::kOff);
		}
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

		frc::SmartDashboard::PutData("Field", &driveSim.GetField());
	}

private:
	DrivebaseSimFX driveSim{front_left, front_right, pigeon};
};

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif