package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.Robot;
import org.usfirst.frc.team4153.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;

public class Forklift implements Subsystem {

	private CANTalon liftMotor, liftMotor2, forkMotor, brakeMotor;

	//Difference between lift motor position and desired position (in encoder value) that is considered "close enough" to the desired position
	private final int BRAKE_TOLERANCE = 3; //Set later


	/** 
	 * when this is true, the motor will calibrate on the next loop
	 */
	protected boolean needsReset = false;

	/** Calibration thread.  Null unless calibrating. */
	protected CalibrateThread calibrateThread;

	/**
	 * Sets up the lift, fork, and brake motors and the manipulator joystick
	 */
	public void init() {


		liftMotor = new CANTalon( RobotMap.LIFT_MOTOR );
		liftMotor.changeControlMode( CANTalon.ControlMode.Position);	
		liftMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		liftMotor.setPosition(0);  
		liftMotor.setPID(RobotMap.LIFT_MOTOR_P, RobotMap.LIFT_MOTOR_I, RobotMap.LIFT_MOTOR_D, RobotMap.LIFT_MOTOR_FEED_FORWARD, 100, RobotMap.LIFT_MOTOR_RAMP, 0);   		//magical numbers...manual
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
		brakeMotor.setPID(RobotMap.BRAKE_MOTOR_P, RobotMap.BRAKE_MOTOR_I, RobotMap.BRAKE_MOTOR_D, RobotMap.BRAKE_MOTOR_FEED_FORWARD, 100, RobotMap.BRAKE_MOTOR_RAMP, 0);
		brakeMotor.setProfile(0);
		brakeMotor.ClearIaccum();

		calibrate();
	}

	public void reset() {
		liftMotor.setPosition( 0.0 );
		forkMotor.setPosition(0.0);
	}

	
	public void moveLift(double position) {
		//if (!isCalibrating) {
			liftMotor.set(position);
		//}
	}

	/** 
	 * Resets the lift motor to the lowest position and sets that as 0
	 */
	public void calibrate() {
		if (calibrateThread == null ) {
			calibrateThread = new CalibrateThread();
			calibrateThread.start();
		}

	}

	private class CalibrateThread extends Thread {
		public void run() {
			try {
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
			} finally {
				calibrateThread = null;
				needsReset = false;
			}
		}
	}

	
	public void iterate() {
		
		//boolean buttonIsPressed = SmartDashboard.getBoolean("PresetButtonPressed");
		double desired;
		double current;
		
		if( Robot.getRobot().getManipulatorJoystick().getRawButton( 3 ) == true ) {
			needsReset = true;
		}
		
		if (needsReset) {
			calibrate();
		}
		
		
		
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


		//desired = ( manipulatorJoystick.getY() * RobotMap.CLICKS_PER_ROTATION * 3 ) + liftMotor.getPosition();  //test code 
		//desired = ( Robot.getRobot().getManipulatorJoystick().getY() * 200 ) + liftMotor.getPosition();  //test code
		
		current = Robot.getRobot().getManipulatorJoystick().getY();
		
		if( Math.abs( current ) > 0.1 ) {
			desired = ( current > 0.0 ) ? 50 : 7770;		// min position should be zero.... max position should be 7810
			liftMotor.enableControl();
			iterateBrakeMotor( false );
			moveLift( desired );
		} else {
			desired = current;
			iterateBrakeMotor( true );
			liftMotor.disableControl();
		}

		//System.out.println( "Wanted Value: " + desired + "\t    Lift motor position: " + liftMotor.getPosition() );
		
		
		
		
//		public static final double LIFT_MOTOR_P = 1.0;
//		public static final double LIFT_MOTOR_I = .001;
//		public static final double LIFT_MOTOR_D = 1.0;
//		public static final double LIFT_MOTOR_FEED_FORWARD = .001;
//		public static final int LIFT_MOTOR_RAMP = 36;

		liftMotor.setPID( 0.5, 0.001, 0.0, 0.001, 0, 36, 0 ); ////!!!!!!!!!!!!!!!!!!! IMPORTANT.... 

	}


	public void iterateBrakeMotor( boolean brakeBoolean) {
		
		if ( brakeBoolean ) {
			brakeMotor.set(0.3);
		}
		else {
			brakeMotor.set(-0.3);
		}
		
		
		/*
		if( Robot.getRobot().getManipulatorJoystick().getRawButton( 4 ) ) {
			brakeMotor.set( 0.3 );
		} 
		if( Robot.getRobot().getManipulatorJoystick().getRawButton( 5 ) ) {
			brakeMotor.set( -0.3 );
		}
		*/
		
	}
	
	

}
