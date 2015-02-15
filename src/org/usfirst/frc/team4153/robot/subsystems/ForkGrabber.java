package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.Robot;
import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.CANJaguar.ControlMode;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ForkGrabber implements Subsystem {

	private CANTalon forkMotor;

	/**
	 * Sets up the lift, fork, and brake motors and the manipulator joystick
	 */
	public void init() {

		forkMotor = new CANTalon(RobotMap.FORK_MOTOR);
		forkMotor.changeControlMode(CANTalon.ControlMode.Position);
		forkMotor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		forkMotor.setPosition(0);
		forkMotor.setPID( 0.5, 0.001, 1 );
		forkMotor.setProfile(0);
		forkMotor.ClearIaccum();

	}

	public void reset() {
		forkMotor.setPosition(0.0);
	}


	/**
	 * Called periodically
	 */
	public void iterate() {
		forkMotor.setPID( 25.0, 0.05, 0 );
		
		forkMotor.reverseSensor( false );
		forkMotor.reverseOutput( true );
		
		if (Robot.getRobot().getManipulatorJoystick().getRawButton(8)) {
			forkMotor.set( 534 );													//max value is 534 ( closed fork ), min value is 490 ( open fork )
		}
		if (Robot.getRobot().getManipulatorJoystick().getRawButton(9)) {
			forkMotor.set( 490 );
		}
		if (Robot.getRobot().getManipulatorJoystick().getRawButton( 11 )) {
			forkMotor.set( 515 );
		}
		
		//System.out.println( "Flex Sensor Position: " + forkMotor.getAnalogInRaw() );
		
	}

	

}
