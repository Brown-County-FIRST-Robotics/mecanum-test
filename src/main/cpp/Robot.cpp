/*  
	TODO:
	move motor varibles to struct
	remove analog_output
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
#include "frc/AnalogOutput.h"

using namespace frc;

class Robot : public TimedRobot
{
public:
	WPI_TalonFX back_left{1};
	double back_left_target_pos=0;
	double back_left_pos=0;
	double back_left_previous_pos=0;

	WPI_TalonFX front_left{0};
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
	int intake_solenoid_previous_position = shooter_solenoid.kForward;
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

	AnalogOutput debugging_out{0};
	
	XboxController controller{0};

	GenericHID hid{0};
	
	float driveSpeed = 0.5;
	float shooter_angle_speed = 1;
	float shooter_speed = 1.0;
	bool is_birb_activated = true;
	bool hub_shooting[2] = {false,false}; // [isHubShooting, isMovingToCorrectAngle]
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

	Timer autonomous_timer;
	bool autonomous_shooting=false;

	void SimulationPeriodic()
	{
	}

	void TeleopPeriodic()
	{
		double joyX = -controller.GetLeftX();
		double joyR = -controller.GetLeftY();
		double joyY = controller.GetRightX();

		/* deadband gamepad 5% */
		if (fabs(joyR) < 0.2)
			joyR = 0;
		if (fabs(joyX) < 0.2)
			joyX = 0;
		if (fabs(joyY) < 0.2)
			joyY = 0;

		mecDrive.DriveCartesian(joyY*driveSpeed, joyX*driveSpeed, joyR*driveSpeed);

	
		if(controller.GetRightBumper()){
			shooter_angle.Set(shooter_angle_speed);
		}else if(controller.GetLeftBumper() && potentiometer.GetValue()>95){
			shooter_angle.Set(-shooter_angle_speed);
		}else{
			shooter_angle.Set(0);
		}
		

		if(!hub_shooting[0]){
			if(controller.GetRightTriggerAxis()>controller.GetLeftTriggerAxis()){
				shooter.Set(controller.GetRightTriggerAxis()*shooter_speed);
				intake.Set(controller.GetRightTriggerAxis()*shooter_speed);
			}

			if(controller.GetRightTriggerAxis()<controller.GetLeftTriggerAxis()){
				shooter.Set(controller.GetLeftTriggerAxis()*-0.5);
				intake.Set(controller.GetLeftTriggerAxis()*-0.5);
			}

			if(controller.GetRightTriggerAxis()==controller.GetLeftTriggerAxis()){
				shooter_top.Set(0);
				shooter_bottom.Set(0);
				intake.Set(0);
			}	
		}

		if(controller.GetAButtonPressed()){
			shooter_solenoid_timer.Start();
			shooter_solenoid.Set(DoubleSolenoid::kForward);
		}

		if(shooter_solenoid_timer.HasElapsed(.1_s) && .2_s>shooter_solenoid_timer.Get()){
			shooter_solenoid.Set(DoubleSolenoid::kOff);
		}

		if(shooter_solenoid_timer.HasElapsed(.5_s) && .55_s>shooter_solenoid_timer.Get()){
			shooter_solenoid.Set(DoubleSolenoid::kReverse);
		}

		if(shooter_solenoid_timer.HasElapsed(.6_s) && .7_s>shooter_solenoid_timer.Get()){
			shooter_solenoid_timer.Stop();
			shooter_solenoid_timer.Reset();
			shooter_solenoid.Set(DoubleSolenoid::kOff);
		}

		if(controller.GetBButtonPressed() && is_birb_activated){
			climber_solenoid_timer.Start();
			if(climber_solenoid_previous_position == DoubleSolenoid::kForward){
				climber_solenoid.Set(DoubleSolenoid::kReverse);
			}
			if(climber_solenoid_previous_position == DoubleSolenoid::kReverse){
				climber_solenoid.Set(DoubleSolenoid::kForward);
			}
		}

		if(climber_solenoid_timer.HasElapsed(.25_s)){
			climber_solenoid_timer.Stop();
			climber_solenoid_timer.Reset();
			climber_solenoid_previous_position=climber_solenoid.Get();
			climber_solenoid.Set(DoubleSolenoid::kOff);
		}

		if(controller.GetStartButtonPressed()){
			intake_solenoid_timer.Start();
			if(intake_solenoid_previous_position==DoubleSolenoid::kForward){
				intake_solenoid.Set(DoubleSolenoid::kReverse);
			}

			if(intake_solenoid_previous_position==DoubleSolenoid::kReverse){
				intake_solenoid.Set(DoubleSolenoid::kForward);
			}		
		}

		if(intake_solenoid_timer.HasElapsed(.25_s)){
			intake_solenoid_timer.Stop();
			intake_solenoid_timer.Reset();
			intake_solenoid_previous_position=intake_solenoid.Get();
			intake_solenoid.Set(DoubleSolenoid::kOff);
		}

		if(controller.GetXButtonPressed()){
			hub_shooting[0]=!hub_shooting[0];
			hub_shooting[1]=!hub_shooting[1];
		}
		/*        UNCOMMENT THIS ONCE POTENTIOMETER IS FIXED
		if(hub_shooting[0]){
			if(hub_shooting[1]){
				double potentiometer_val = static_cast<double>(potentiometer.GetValue());
				if(((potentiometer_val-2554.0)*(90.0/448.0))<55){
					shooter_angle.Set(1);
				}else if(((potentiometer_val-2554.0)*(90.0/448.0))>60){
					shooter_angle.Set(-1);
				}else{
					shooter_angle.Set(0);
					hub_shooting_timer.Start();
					hub_shooting[1]=false;
					shooter.Set(.3);
				}
			}

			if(hub_shooting_timer.HasElapsed(1_s) && hub_shooting_timer.Get()<1.1_s){
				shooter_solenoid.Set(DoubleSolenoid::kForward);
			}else if(hub_shooting_timer.HasElapsed(1.25_s) && hub_shooting_timer.Get()<1.35_s){
				shooter_solenoid.Set(DoubleSolenoid::kOff);
			}else if(hub_shooting_timer.HasElapsed(1.75_s) && hub_shooting_timer.Get()<1.85_s){
				shooter_solenoid.Set(DoubleSolenoid::kReverse);
			}else if(hub_shooting_timer.HasElapsed(2_s) && hub_shooting_timer.Get()<2.1_s){
				shooter_solenoid.Set(DoubleSolenoid::kOff);
			}else if(hub_shooting_timer.HasElapsed(2.75_s) && hub_shooting_timer.Get()<2.8_s){
				shooter_solenoid.Set(DoubleSolenoid::kForward);
			}else if(hub_shooting_timer.HasElapsed(2.85_s) && hub_shooting_timer.Get()<2.95_s){
				shooter_solenoid.Set(DoubleSolenoid::kOff);
			}else if(hub_shooting_timer.HasElapsed(2.25_s) && hub_shooting_timer.Get()<2.3_s){
				shooter_solenoid.Set(DoubleSolenoid::kReverse);
			}else if(hub_shooting_timer.HasElapsed(2.35_s) && hub_shooting_timer.Get()<2.45_s){
				hub_shooting_timer.Reset();
				hub_shooting_timer.Stop();
				shooter_solenoid.Set(DoubleSolenoid::kOff);
				shooter.Set(0);
				shooter_angle.Set(-1);
			}
			if(((potentiometer.GetValue()-2554)*(90/448))<-5 && !hub_shooting[1]){
				shooter_angle.Set(0);
				hub_shooting[0]=false;
			}
		}*/

		if(controller.GetBackButtonPressed()){
			shooter.Set(0);
			shooter_angle.Set(0);
			hub_shooting[0]=false;
			hub_shooting_timer.Stop();
			hub_shooting_timer.Reset();
			shooter_solenoid.Set(DoubleSolenoid::kOff);
		}

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

	void AutonomousInit(){
		autonomous_timer.Start();
		front_left.Set(-0.3);
		front_right.Set(-0.3);
		back_left.Set(-0.3);
		back_right.Set(-0.3);
		intake_solenoid_timer.Start();
		climber_solenoid_timer.Start();
	}

	void AutonomousPeriodic(){
		if(intake_solenoid_timer.Get()>10_s && intake_solenoid_timer.Get() < 10.1_s){
			intake_solenoid.Set(DoubleSolenoid::kForward);
		}
		if(climber_solenoid_timer.Get()>10_s && climber_solenoid_timer.Get() < 10.1_s){
			climber_solenoid.Set(DoubleSolenoid::kForward);
		}
		if(intake_solenoid_timer.Get()>10.25_s && intake_solenoid_timer.Get() < 10.35_s){
			intake_solenoid.Set(DoubleSolenoid::kOff);
			intake_solenoid_timer.Stop();
			intake_solenoid_timer.Reset();
		}
		if(climber_solenoid_timer.Get()>10.25_s && climber_solenoid_timer.Get() < 10.35_s){
			climber_solenoid.Set(DoubleSolenoid::kOff);
			climber_solenoid_timer.Stop();
			climber_solenoid_timer.Reset();
		}

		if(autonomous_timer.HasElapsed(2_s)){
			front_left.Set(0);
			front_right.Set(0);
			back_left.Set(0);
			back_right.Set(0);
		}/*else{
			front_left.Set(-0.3);
			front_right.Set(-0.3);
			back_left.Set(-0.3);
			back_right.Set(-0.3);
		}*/
	}

private:
	
};

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif