/*  
	TODO:
	move motor varibles to struct
*/
#include <iostream>
#include <string>

#include "frc/TimedRobot.h"
#include "frc/Joystick.h"
#include "frc/Solenoid.h"
#include "ctre/Phoenix.h"
#include "frc/motorcontrol/MotorControllerGroup.h"
#include "frc/drive/MecanumDrive.h"
#include "frc/DoubleSolenoid.h"
#include "frc/Timer.h"
#include "frc/PneumaticsControlModule.h"
#include "frc/GenericHID.h"
#include "units/time.h"
#include "frc/XboxController.h"
#include "frc/AnalogInput.h"

using namespace frc;

class Robot : public TimedRobot
{
public:
	WPI_TalonFX back_left{0};
	double back_left_target_pos=0;
	double back_left_pos=0;
	double back_left_previous_pos=0;
	WPI_TalonFX front_left{1};
	double front_left_target_pos=0;
	double front_left_pos=0;
	double front_left_previous_pos=0;
	WPI_TalonFX front_right{2};
	double front_right_target_pos=0;
	double front_right_pos=0;
	double front_right_previous_pos=0;
	WPI_TalonFX back_right{3};
	double back_right_target_pos=0;
	double back_right_pos=0;
	double back_right_previous_pos=0;
	WPI_TalonFX shooter_top{6};
	WPI_TalonFX shooter_bottom{7};
	WPI_TalonSRX shooter_angle_1{10};
	WPI_TalonSRX shooter_angle_2{11};
	WPI_TalonFX intake{12};
	double intake_target_pos=0;
	double intake_pos=0;
	double intake_previous_pos=0;

	DoubleSolenoid shooter_solenoid{PneumaticsModuleType::CTREPCM, 0, 1};
	Timer shooter_solenoid_timer;

	DoubleSolenoid intake_solenoid{PneumaticsModuleType::CTREPCM, 2, 3};
	int shooter_solenoid_previous_position = shooter_solenoid.kForward;
	Timer intake_solenoid_timer;

	DoubleSolenoid climber_solenoid{PneumaticsModuleType::CTREPCM, 4, 5};
	int climber_solenoid_previous_position = shooter_solenoid.kReverse;
	Timer climber_solenoid_timer;

	MotorControllerGroup shooter_angle{shooter_angle_2, shooter_angle_1};
	double shooter_angle_target_pos=0;
	double shooter_angle_pos=0;
	double shooter_angle_previous_pos=0;
	MotorControllerGroup shooter{shooter_bottom, shooter_top};
	double shooter_target_pos=0;
	double shooter_pos=0;
	double shooter_previous_pos=0;

	

	XboxController controller{0};

	GenericHID hid{0};
	
	float driveSpeed = 0.5;
	float shooter_angle_speed = 1;
	float shooter_speed = .35;
	bool is_birb_activated = true;
	bool hub_shooting[2] = {true,true}; // [isHubShooting, isMovingToCorrectAngle]
	Timer hub_shooting_timer;
	AnalogInput potentiometer{0};

	Timer timer;
	Timer shooter_timer;

	MecanumDrive mecDrive{front_left, back_left, front_right, back_right};

	bool is_shooting=false;
	double shooter_step=0;
	bool lock_on_mode=false;
	Timer shooting_timer;
	bool firing=false;

	void SimulationPeriodic()
	{
	}

	void TeleopPeriodic()
	{
		double joyX = controller.GetLeftX();
		double joyY = -controller.GetLeftY();
		double joyR = controller.GetRightX();

		/* deadband gamepad 5% */
		if (fabs(joyR) < 0.05)
			joyR = 0;
		if (fabs(joyX) < 0.05)
			joyX = 0;
		if (fabs(joyY) < 0.05)
			joyY = 0;
		mecDrive.DriveCartesian(joyY, joyX, joyR);

	}

	void RobotInit()
	{
		timer.Start();
		front_left.SetNeutralMode(Brake);
		back_left.SetNeutralMode(Brake);
		front_right.SetNeutralMode(Brake);
		back_right.SetNeutralMode(Brake);
		shooter_top.SetInverted(true);
		shooter_angle_1.SetInverted(true);
		shooter_angle_2.SetInverted(true);

	}

private:
	
};

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif