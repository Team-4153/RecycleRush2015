package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.Robot;
import org.usfirst.frc.team4153.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ForkGrabber implements Subsystem {

	private CANTalon forkMotor;
	private long currentTime;
	private double lastValueOfWantedPosition;
	double wantedPositionOfGrabber;
	private boolean wantedOpened = true;   	//true means open

	/**
	 * Sets up the lift, fork, and brake motors and the manipulator joystick
	 */
	public void reset() { }
	
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
		
		lastValueOfWantedPosition = SmartDashboard.getNumber( "ForkOpening" );

	}

	/**
	 * Called periodically
	 */
	public void iterate() {
		
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
		
		if( wantedOpened ) {
			forkMotor.enableControl();
			forkMotor.set( 0.5 );
			currentTime = System.currentTimeMillis();
		} else {
			forkMotor.enableControl();
			forkMotor.set( -0.5 );
			currentTime = System.currentTimeMillis();
			checkMotorTimeout();

		}
		
		

		// System.out.println( "Flex Sensor Position: " + forkMotor.getAnalogInRaw() + ", Motor Setpoint, " + forkMotor.getSetpoint() + ", Motor output: " + forkMotor.getOutputCurrent() );

		System.out.println ("Wanted: " + wantedPositionOfGrabber  +", Current: "+ forkMotor.getPosition()+" ForwardLimit: "+forkMotor.isFwdLimitSwitchClosed()+" Back: "+forkMotor.isRevLimitSwitchClosed());
		
		
	
	}



	public void checkMotorTimeout() {
		SmartDashboard.putNumber("GrabberPosition", forkMotor.getAnalogInRaw());

		if ( (System.currentTimeMillis() - currentTime >= 1000) || forkMotor.getOutputCurrent()  >= 5.5) {			//GOES THROUGH LOOP
			forkMotor.set( -0.15 );
		}
	}
	
	public void moveTo(double position) {
		forkMotor.enableControl();
		if (forkMotor.getPosition() < position) {
			forkMotor.set(0.6);
		}
		else {
			forkMotor.set(-0.6);
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



}
