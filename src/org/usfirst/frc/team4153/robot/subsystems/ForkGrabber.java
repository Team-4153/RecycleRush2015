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
	private long currentTime;

	/**
	 * Sets up the lift, fork, and brake motors and the manipulator joystick
	 */
	public void init() {

		forkMotor = new CANTalon(RobotMap.FORK_MOTOR);
		
		forkMotor.clearStickyFaults();
		
		forkMotor.changeControlMode(CANTalon.ControlMode.Position);
		forkMotor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		forkMotor.setPosition(0);
		forkMotor.setPID( 0.5, 0.001, 1 );
		forkMotor.setProfile(0);
		forkMotor.ClearIaccum();
		forkMotor.setSafetyEnabled( true );
		forkMotor.enableBrakeMode( true );
		
	}

	public void reset() {
		forkMotor.setPosition(0.0);
	}


	/**
	 * Called periodically
	 */
	public void iterate() {
		forkMotor.setPID( 65.0, 0.27, 0 );
		
		forkMotor.reverseSensor( false );
		forkMotor.reverseOutput( true );
		
		if (forkMotor.getOutputCurrent()>=1.0) {
			forkMotor.set(forkMotor.getAnalogInRaw());
			forkMotor.disable();
		}
		
		
		if (Robot.getRobot().getManipulatorJoystick().getRawButton(8)) {
			forkMotor.enableControl();
			forkMotor.set( 523 );					
			currentTime = System.currentTimeMillis(); //max value is 523 ( closed fork ), min value is 490 ( open fork )
		}
		if (Robot.getRobot().getManipulatorJoystick().getRawButton(9)) {
			forkMotor.enableControl();
			forkMotor.set( 490 );
			currentTime = System.currentTimeMillis();
		}
		if (Robot.getRobot().getManipulatorJoystick().getRawButton( 11 )) {
			forkMotor.enableControl();
			forkMotor.set( 505 );
			currentTime = System.currentTimeMillis();
		}
		
		if (System.currentTimeMillis() - currentTime >= 1000 || forkMotor.getOutputCurrent()  >= 5.5) {			//GOES THROUGH LOOP
			forkMotor.set( forkMotor.getAnalogInRaw() );
			forkMotor.disable();
			forkMotor.disableControl();
		}
		
		System.out.println( "Flex Sensor Position: " + forkMotor.getAnalogInRaw() + ", Motor Setpoint, " + forkMotor.getSetpoint() + ", Motor output: " + forkMotor.getOutputCurrent() );

		if (forkMotor.isControlEnabled()) {
			System.out.println("Control enabled");
		} else {
			System.out.println("Control disabled");
		}
	}

	

}
