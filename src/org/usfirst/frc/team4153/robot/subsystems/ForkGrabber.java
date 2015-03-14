package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Subsystem for controlling the fork grabber
 */
public class ForkGrabber implements Subsystem {

	private CANTalon forkMotor;
	private long currentTime;
	double wantedPositionOfGrabber;
	private boolean wantedOpened = true;   	//true means open
	private boolean oldWantedOpen = false;

	/**
	 * Sets up the lift, fork, and brake motors and the manipulator joystick
	 */
	

	public void init() {

		forkMotor = new CANTalon(RobotMap.FORK_MOTOR);

		forkMotor.clearStickyFaults();

		forkMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		forkMotor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		forkMotor.setPosition(0);
		forkMotor.setPID( 65.0, 0.27, 0 );
		forkMotor.setProfile(0);
		forkMotor.ClearIaccum();
		forkMotor.setSafetyEnabled( false );
		forkMotor.enableBrakeMode( true );

		forkMotor.reverseSensor( false );
		forkMotor.reverseOutput( true );

	

		/*forkMotor.enableForwardSoftLimit(true);
		forkMotor.setForwardSoftLimit(523);

		forkMotor.enableReverseSoftLimit(true);
		forkMotor.setReverseSoftLimit(490);
		 */

		oldWantedOpen = SmartDashboard.getBoolean( "OpenFork", true );

	}

	/**
	 * Called periodically during teleop
	 */
	public void iterate() {


		//Gets whether or not the grabber should be open from the driver station
		wantedOpened = SmartDashboard.getBoolean( "OpenFork", true );

		//		if ( Robot.getRobot().getManipulatorJoystick().getRawButton(8) ) {
		//			//max value is 523 ( closed fork ), min value is 490 ( open fork )
		//			setPosition(530); 
		//		}
		//		if ( Robot.getRobot().getManipulatorJoystick().getRawButton(9) ) {
		//			setPosition(490); 
		//		}
		//		if ( Robot.getRobot().getManipulatorJoystick().getRawButton( 11 ) ) {
		//			setPosition(505); 
		//		}

		//Opens or closes depending
		
		if( wantedOpened ) {
			open();
		} else {
			close();
		}
		//If the desired position has changed from last iteration, start the timer for the timeout check
		if (oldWantedOpen != wantedOpened) {
			currentTime = System.currentTimeMillis();
		}
		checkMotorTimeout();

		oldWantedOpen = wantedOpened;

		
	}

	// TODO this code all assumes that positive on the right spinner means
	// forward. This might or might not work. It also might not matter. 

	// Note that the left spin motor output is reversed.

	/**
	 * controls the spinning motors on the end of the arms: forward means out
	 */


	public void close() {
		forkMotor.enableControl();
		forkMotor.set( -1 ); //Closes faster than it opens
	}

	public void open() {
		forkMotor.enableControl();
		forkMotor.set( 0.5 );
	}

	/**
	 * If it has been at least a second, and the motor still hasn't closed, or if the current is too high, slow motor down to prevent burning out
	 */
	public void checkMotorTimeout() {
		SmartDashboard.putNumber("GrabberPosition", forkMotor.getAnalogInRaw());
		if ( checkStalling()) {			//GOES THROUGH LOOP
			forkMotor.set( -0.1 );
			//System.out.println("Lessening grabber output");
		}
	}

	/**
	 * @return true if the motor is probably stalling
	 */
	public boolean checkStalling() {
		return (System.currentTimeMillis() - currentTime >= 1000) || forkMotor.getOutputCurrent()  >= 5.5;
	}

	/**
	 * (No longer used) Moves motor to given position (in encoder ticks)
	 */
	public void moveTo(double position) {
		forkMotor.enableControl();
		if (forkMotor.getPosition() < position) {
			forkMotor.set(0.6);
		}
		else {
			forkMotor.set(-0.8);
		}
		currentTime = System.currentTimeMillis();

	}

	/**
	 * 
	 * @return whether the fork is within tolerance of its setpoint
	 */
	public boolean inPosition() {
		return Math.abs( forkMotor.getPosition() - wantedPositionOfGrabber ) < RobotMap.FORK_POSITION_TOLERANCE;
	}

	/**
	 * This method will return true ONLY when the fork is finished closing
	 * @return Whether the fork is closed
	 */
	public boolean isClosed () {
		//return ( inPosition() && forkMotor.getAnalogInRaw() >= 519 );		//we have 2 positions that it closes in, so only in position should matter -nikola
		return ( inPosition()  );
	}

	public void reset() { } //Does nothing, but has to be here because we're implementing the subsystem interface

}
