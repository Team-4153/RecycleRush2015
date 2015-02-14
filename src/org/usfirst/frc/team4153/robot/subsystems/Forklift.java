package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.Robot;
import org.usfirst.frc.team4153.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;

public class Forklift implements Subsystem {

	private CANTalon liftMotor, liftMotor2, forkMotor, brakeMotor;

	//Difference between lift motor position and desired position (in encoder value) that is considered "close enough" to the desired position
	private final int BRAKE_TOLERANCE = 3; //Set later

	/**
	 * The lift motor "locks" to stop interference or unexpected behavior while calibrating
	 */
	protected boolean resetLock = false;

	/** 
	 * when this is true, the motor will calibrate on the next loop
	 */
	protected boolean needsReset = false;



	/**
	 * Sets up the lift, fork, and brake motors and the manipulator joystick
	 */
	public void init() {


		liftMotor = new CANTalon( RobotMap.LIFT_MOTOR );
		liftMotor.changeControlMode( CANTalon.ControlMode.Position);	
		liftMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		liftMotor.setPosition(0);  
		liftMotor.setPID(1.0, .001, 1, .001, 100, 36, 0);   		//magical numbers...manual
		liftMotor.setProfile(0);
		liftMotor.ClearIaccum();
		liftMotor.reverseSensor(true);
		liftMotor.reverseOutput( false );
		liftMotor.enableBrakeMode(true);

		liftMotor2 = new CANTalon( RobotMap.LIFT_MOTOR2);
		liftMotor2.changeControlMode(CANTalon.ControlMode.Follower);
		liftMotor2.set( liftMotor.getDeviceID() );


		brakeMotor = new CANTalon(RobotMap.BRAKE_MOTOR);
		brakeMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		brakeMotor.setPosition(0);
		brakeMotor.setPID(0.1, 0.001, 1, 0.001, 100, 36, 0);
		brakeMotor.setProfile(0);
		brakeMotor.ClearIaccum();

		//iterateForkMotor(534);
		calibrate();
	}

	public void reset() {
		liftMotor.setPosition( 0.0 );
		forkMotor.setPosition(0.0);
	}

	/**
	 * Moves the lift motor
	 * @param position
	 * The position according to the Talon to move to
	 */
	public void moveLift(double position) {
		//if (!resetLock) {
			liftMotor.set(position);
		//}
	}

	/** 
	 * Resets the lift motor to the lowest position and sets that as 0
	 */
	public void calibrate() {
		if (!resetLock) {
			resetLock = true;
			CalibrateThread calibrateThread = new CalibrateThread();
			calibrateThread.start();
		}

	}

	private class CalibrateThread extends Thread {
		public void run() {
			liftMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
			while (!liftMotor.isRevLimitSwitchClosed()) {
				liftMotor.set(-0.1);
				try {
					sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			liftMotor.set(0);
			liftMotor.changeControlMode(CANTalon.ControlMode.Position);
			liftMotor.setPosition(0);
			resetLock = false;
			needsReset = false;
		}
	}

	/**
	 * Called periodically
	 */
	public void iterateLiftMotor() {



		//boolean buttonIsPressed = SmartDashboard.getBoolean("PresetButtonPressed");
		double desired;
		/*
		if (needsReset) {
			calibrate();
		}
		*/
		
		
		/*																						//actual code to be used
		if( Math.abs(manipulatorJoystick.getY()) < 0.05 ) {
			if( buttonIsPressed ) {
				desired = SmartDashboard.getNumber("PresetValue") + SmartDashboard.getNumber("OffsetValue") + SmartDashboard.getNumber("ContainerOffset");
			} else {
				desired = liftMotor.getPosition();
			}

		} else {
			desired = ( manipulatorJoystick.getY() * RobotMap.CLICKS_PER_ROTATION * 3 ) + liftMotor.getPosition();
			SmartDashboard.putBoolean("PresetButtonPressed", false);
		}
		 */


		liftMotor.setPID(1.0, .001, 1, .001, 100, 36, 0);   

		//desired = ( manipulatorJoystick.getY() * RobotMap.CLICKS_PER_ROTATION * 3 ) + liftMotor.getPosition();  //test code 
		desired = ( Robot.getRobot().getManipulatorJoystick().getY() * 200 ) + liftMotor.getPosition();  //test code 

		System.out.println( "Wanted Value: " + desired + "\t    Lift motor position: " + liftMotor.getPosition() );
		if (Math.abs(liftMotor.get() - desired) <= BRAKE_TOLERANCE) {
		    iterateBrakeMotor(true);
			liftMotor.disableControl();
		}
		else {
			liftMotor.enableControl();
			moveLift(desired);	
		    iterateBrakeMotor(false);
		}

	}


	public void iterateBrakeMotor( boolean brakeBoolean) {

		if( brakeMotor.isRevLimitSwitchClosed() || brakeMotor.isFwdLimitSwitchClosed() ) {
			brakeMotor.set( 0.0 );
		}
		else {
			if (brakeBoolean) {											//Code to activate brake
				brakeMotor.set( 0.3 );					//positive activates brake --- reverse limit 
			} else {													//Code to release brake
				brakeMotor.set( -0.3 );					//negative deactivates brake --- forward limit
			}
		}
	}

	@Override
	public void iterate() {
		iterateLiftMotor();		
	}
}
