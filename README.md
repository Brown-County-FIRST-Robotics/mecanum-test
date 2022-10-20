# mecanum-test
This is a combination of the launcher.py and robot.py files in the auto_aim branch of last year's code. 

|Old name|New name|
|-|-|
|self.shooter_dictionary[“shooting”]|is_shooting|
|self.shooter_dictionary[“step”]|shooter_step|
|self.shooter_dictionary[“timer”]|shooting_timer|
|self.shooter_dictionary[“firing”]|firing|
|self.shooter_dictionary[“lock_on_mode”]|lock_on_mode|
|self.motor_dictionary[{motor name}][“motor”]|{motor name}|
|self.motor_dictionary[{motor name}][“target_position”]|{motor name}_target_pos|
|self.motor_dictionary[{motor name}][“previous_position”]|{motor name}_previous_pos|
|self.motor_dictionary[{motor name}][“motor_position”]|{motor name}_pos|
|self.timer (from autonomousInit)|autonomous_timer|
|wpilib.DoubleSolenoid.Value.kForward|DoubleSolenoid::kForward|
|wpilib.DoubleSolenoid.Value.kOff|DoubleSolenoid::kOff|
|wpilib.DoubleSolenoid.Value.kReverse|DoubleSolenoid::kReverse|

