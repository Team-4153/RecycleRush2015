package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.CANJaguar.ControlMode;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

public class Forklift extends Subsystem {
	
	private CANTalon liftMotor;
	private Joystick manipulatorJoystick;
	
	public Forklift() {
		
		liftMotor = new CANTalon( RobotMap.LIFT_MOTOR );
		liftMotor.changeControlMode( CANTalon.ControlMode.Position);	
		liftMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		liftMotor.setPosition(0);
		liftMotor.ClearIaccum();
		
		manipulatorJoystick.Joystick();
	
	}
	public void moveLiftMotor( double wantedPosition) {

		liftMotor.setPosition(wantedPosition);
		
	}
}
