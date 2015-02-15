package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.Robot;
import org.usfirst.frc.team4153.robot.RobotMap;

import edu.wpi.first.wpilibj.CANTalon;

public class Forklift implements Subsystem {

	private CANTalon liftMotor;
	private CANTalon liftMotor2;
	private CANTalon forkMotor;
	private CANTalon brakeMotor;

	//Difference between lift motor position and desired position (in encoder value) that is considered "close enough" to the desired position
	private final int BRAKE_TOLERANCE = 3; //Set later
	private long counter = 0;
	private long currentTime;


	/** 
	 * when this is true, the motor will calibrate on the next loop
	 */
	//protected boolean needsReset = false;

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
		
		liftMotor.setPID(RobotMap.LIFT_MOTOR_P, RobotMap.LIFT_MOTOR_I, RobotMap.LIFT_MOTOR_D, RobotMap.LIFT_MOTOR_FEED_FORWARD, 0, RobotMap.LIFT_MOTOR_RAMP, 0);
		liftMotor.setProfile(0);
		liftMotor.ClearIaccum();
		liftMotor.reverseSensor(true);
		liftMotor.reverseOutput( false );
		liftMotor.enableBrakeMode( true );
		liftMotor.setReverseSoftLimit( 100 );
		liftMotor.setForwardSoftLimit(7000); 			/// TODO Check and change
		

		liftMotor2 = new CANTalon( RobotMap.LIFT_MOTOR2);
		liftMotor2.changeControlMode(CANTalon.ControlMode.Follower);
		liftMotor2.set( liftMotor.getDeviceID() );


		brakeMotor = new CANTalon(RobotMap.BRAKE_MOTOR);
		brakeMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		brakeMotor.setPosition(0);
		brakeMotor.setPID(RobotMap.BRAKE_MOTOR_P, RobotMap.BRAKE_MOTOR_I, RobotMap.BRAKE_MOTOR_D, RobotMap.BRAKE_MOTOR_FEED_FORWARD, 100, RobotMap.BRAKE_MOTOR_RAMP, 0);
		brakeMotor.setProfile(0);
		brakeMotor.ClearIaccum();
		
		counter = 0;
		currentTime = System.currentTimeMillis();
		

		calibrate();			//taken out temporarily
	}

	public void reset() {
		liftMotor.setPosition( 0.0 );
		forkMotor.setPosition(0.0);
	}


	/** 
	 * Resets the lift motor to the lowest position and sets that as 0
	 */
	public void calibrate() {
		if (calibrateThread == null ) {
			applyBrake( false );
			calibrateThread = new CalibrateThread();
			calibrateThread.start();
		}

	}

	private class CalibrateThread extends Thread {
		public void run() {
			try {
				liftMotor.enableReverseSoftLimit(false);
				liftMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
				int counter = 500;
				while (counter-- > 0 && !liftMotor.isRevLimitSwitchClosed()) {
					liftMotor.set(-0.1);
					System.out.println("Calibrating");
					try {
						sleep(10);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
				
				
			} finally {
				liftMotor.set(0);
				liftMotor.changeControlMode(CANTalon.ControlMode.Position);
				liftMotor.setPosition(0);
				
				calibrateThread = null;
				liftMotor.enableReverseSoftLimit(true);
				System.out.println( "Finished Calibrating" );
			}
		}
	}


	public void iterate() {
		if( Robot.getRobot().getManipulatorJoystick().getRawButton( 3 ) == true ) {
			calibrate();
		} else if (calibrateThread == null) {
			iterateLift();
		}

	}
	
	/** run the lifting code */
	public void iterateLift() {

		//boolean buttonIsPressed = SmartDashboard.getBoolean("PresetButtonPressed");
		
		double currentJoystick;
		double currentMotor;

		double desired = 0;
		
		


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



		//		if( Math.abs( current ) > 0.1 ) {
		//			desired = ( current > 0.0 ) ? 7770 : 50;		// min position should be zero.... max position should be 7810
		//			liftMotor.enableControl();
		//			iterateBrakeMotor( false );
		//			moveLift( desired );
		//		} else {
		//			desired = current;
		//			iterateBrakeMotor( true );
		//			liftMotor.disableControl();
		//		}

		
		

		/*
		  
		if( Math.abs( currentJoystick ) > 0.2 ) {
			//int change = (( currentJoystick > 0.0 ) ? 350 : -350 );		// min position should be zero.... max position should be 7810
			//desired = ( currentMotor + change );		
			
			
			
			if( currentJoystick > 0.0 ) {
				desired = currentMotor + 700;
			} else {
				desired = currentMotor - 700;
			}
			
			liftMotor.enableControl();
			applyBrake( false );
			liftMotor.set(desired);
			
			if( counter % 2000 == 0 ) {
				System.out.println( "...1...Current Motor: " + currentMotor + "    Desired: " + desired + "      Current Joystick: " + currentJoystick );
			}
			
			currentTime = System.currentTimeMillis();

		} else {
			
		
			if( System.currentTimeMillis() - currentTime > 1500 ) {
				liftMotor.disableControl();
			}
			desired = currentMotor;
			applyBrake( true );
			liftMotor.set(desired);
			liftMotor.ClearIaccum();
			//liftMotor.disableControl();
			
			if( counter % 2000 == 0 ) {
				System.out.println( "...2...Current Motor: " + currentMotor + "    Desired: " + desired + "      Current Joystick: " + currentJoystick );
			}
		}
		
		*/
		
		currentJoystick = Robot.getRobot().getManipulatorJoystick().getY();
		currentMotor = liftMotor.getPosition();
	//=================================================================================	TEST CODE TODO
		
		desired = currentMotor;
		if( Robot.getRobot().getManipulatorJoystick().getRawButton( 7 ) ) {
			desired = 2000;
		}
		if( Robot.getRobot().getManipulatorJoystick().getRawButton( 6 ) ) {
			desired = 4000;
		}
		
		
		moveToPosition( desired );
	
		
		if( Math.abs( liftMotor.getPosition() - desired ) < 25 ) {
			applyBrake( true );
		} else {
			applyBrake( false );
		}
		
		
		//System.out.println("Target position: " + desired+", actual position: "+liftMotor.getPosition());

		
	//================================================================================




		//System.out.println( "Wanted Value: " + desired + "\t    Lift motor position: " + liftMotor.getPosition() );




		//		public static final double LIFT_MOTOR_P = 1.0;
		//		public static final double LIFT_MOTOR_I = .001;
		//		public static final double LIFT_MOTOR_D = 1.0;
		//		public static final double LIFT_MOTOR_FEED_FORWARD = .001;
		//		public static final int LIFT_MOTOR_RAMP = 36;

		


	}


	private void applyBrake( boolean brakeBoolean) {

		if ( brakeBoolean ) {
			brakeMotor.set(-0.3);
		}
		else {
			brakeMotor.set(0.3);
		}


		
//		if( Robot.getRobot().getManipulatorJoystick().getRawButton( 4 ) ) {
//			brakeMotor.set( 0.4 );
//		} 
//		if( Robot.getRobot().getManipulatorJoystick().getRawButton( 5 ) ) {
//			brakeMotor.set( -0.4 );
//		}
		 

	}
	
	private void moveToPosition( double desired ) {
		liftMotor.set( desired );
	}



}
