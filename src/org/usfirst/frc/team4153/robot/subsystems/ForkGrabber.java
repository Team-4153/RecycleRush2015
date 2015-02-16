package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ForkGrabber implements Subsystem {

	private CANTalon forkMotor;
	private long currentTime;
	private boolean lastValueOfDrop = false;

	/**
	 * Sets up the lift, fork, and brake motors and the manipulator joystick
	 */
	public void init() {

		forkMotor = new CANTalon(RobotMap.FORK_MOTOR);

		forkMotor.clearStickyFaults();

		forkMotor.changeControlMode(CANTalon.ControlMode.Position);
		forkMotor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		forkMotor.setPosition(0);
		forkMotor.setPID( 65.0, 0.27, 0 );
		forkMotor.setProfile(0);
		forkMotor.ClearIaccum();
		forkMotor.setSafetyEnabled( true );
		forkMotor.enableBrakeMode( true );

		/*forkMotor.enableForwardSoftLimit(true);
		forkMotor.setForwardSoftLimit(523);

		forkMotor.enableReverseSoftLimit(true);
		forkMotor.setReverseSoftLimit(490);
		 */

	}



	public void reset() {
		forkMotor.setPosition(0.0);
	}


	/**
	 * Called periodically
	 */
	public void iterate() {


		forkMotor.reverseSensor( false );
		forkMotor.reverseOutput( true );

		// TODO this needs to be fixed so that it also moves up and down
		
		// TODO this might not work at all
		
		if ( SmartDashboard.getBoolean("Drop") ) {
			if (isClosed()) {
				if (currentTime>=500) {
					forkMotor.enableControl();
					forkMotor.set( 490 );					
					currentTime = System.currentTimeMillis();
				}
			} else {
				if (currentTime>=500) {
					forkMotor.enableControl();
					forkMotor.set( 528 );					
					currentTime = System.currentTimeMillis();
				}
			}
			SmartDashboard.putBoolean("Drop", false);
		}
		
		
		
		if( SmartDashboard.getBoolean( "Drop " ) ) {
			if( isClosed() ) {									//decrease RobotMap.FORK_POSITION_TOLERANCE to make more sensetive
				forkMotor.enableControl();						//max value is 523 ( closed fork ), min value is 490 ( open fork )
				forkMotor.set( 490 );
			} else {
				forkMotor.enableControl();
				forkMotor.set( 528 );	
				SmartDashboard.putBoolean( "Drop" ,  false ); 
			}	
		}
		
		
		if( SmartDashboard.getBoolean( "Drop " ) ) {
			//drop
			//first time through after a change in value of Drop
			if( SmartDashboard.getBoolean( "Drop " ) != lastValueOfDrop ) {
				forkMotor.enableControl();
				forkMotor.set( 490 );
				currentTime = System.currentTimeMillis();
			}
			
			//set "Drop" to false
			if( inPosition() ) {
				SmartDashboard.putBoolean( "Drop" , false );
			}
		}

			

		/*if (Robot.getRobot().getManipulatorJoystick().getRawButton(8)) {
			forkMotor.enableControl();
			forkMotor.set( 528 );					
			currentTime = System.currentTimeMillis();								 //max value is 523 ( closed fork ), min value is 490 ( open fork )
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
		 */

		if ( System.currentTimeMillis() - currentTime >= 1000 || forkMotor.getOutputCurrent()  >= 5.5) {			//GOES THROUGH LOOP
			forkMotor.set( forkMotor.getAnalogInRaw() );
			forkMotor.disable();
			forkMotor.disableControl();
			System.out.println("Fork Motor Disabled");
		}

		System.out.println( "Flex Sensor Position: " + forkMotor.getAnalogInRaw() + ", Motor Setpoint, " + forkMotor.getSetpoint() + ", Motor output: " + forkMotor.getOutputCurrent() );

		if (forkMotor.isControlEnabled()) {
			//	System.out.println("Control enabled");
		} else {
			//	System.out.println("Control disabled");
		}
		
		
		lastValueOfDrop = SmartDashboard.getBoolean( "Drop" );
	}

	/**
	 * 
	 * @return whether the fork is within tolerance of its setpoint
	 */
	public boolean inPosition() {
		return Math.abs( forkMotor.getAnalogInRaw() - forkMotor.getSetpoint() ) < RobotMap.FORK_POSITION_TOLERANCE;
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
