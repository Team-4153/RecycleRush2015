package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.Robot;
import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Forklift implements Subsystem {

	private CANTalon liftMotor;
	private CANTalon liftMotor2;
	private CANTalon brakeMotor;

	//Difference between lift motor position and desired position (in encoder value) that is considered "close enough" to the desired position
	private final int BRAKE_TOLERANCE = 3; //Set later
	private long counter = 0;
	private long currentTime;
	private double lastSetPoint = 0;
	
	boolean firstRun = true;


	ForkGrabber forkgrabber;



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
		SmartDashboard.putBoolean( "PresetButtonPressed", false );
		
		
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
		liftMotor.setForwardSoftLimit(8900); 			/// TODO Check and change
		liftMotor.enableForwardSoftLimit(true);
		liftMotor.setSafetyEnabled(false);

		liftMotor2 = new CANTalon( RobotMap.LIFT_MOTOR2);
		liftMotor2.changeControlMode(CANTalon.ControlMode.Follower);
		liftMotor2.set( liftMotor.getDeviceID() );


		brakeMotor = new CANTalon(RobotMap.BRAKE_MOTOR);
		brakeMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		brakeMotor.setPosition(0);
		brakeMotor.setPID(RobotMap.BRAKE_MOTOR_P, RobotMap.BRAKE_MOTOR_I, RobotMap.BRAKE_MOTOR_D, RobotMap.BRAKE_MOTOR_FEED_FORWARD, 100, RobotMap.BRAKE_MOTOR_RAMP, 0);
		brakeMotor.setProfile(0);
		brakeMotor.ClearIaccum();

		forkgrabber = new ForkGrabber();
		forkgrabber.init();

		counter = 0;
		currentTime = System.currentTimeMillis();


		calibrate();			//taken out temporarily
	}

	public void reset() {
		liftMotor.setPosition( 0.0 );
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
	
	public boolean isCalibrating() {
		return !(calibrateThread == null);
	}
	
	/**
	 * the can kept complaining I wasnt updating it enough
	 */
	public void autoPeriodic() {
		liftMotor.set(liftMotor.get());
	}

	public void close() {
		forkgrabber.setPosition(530);
		forkgrabber.checkMotorTimeout();
	}
	
	public void open() {
		forkgrabber.setPosition(490);
		forkgrabber.checkMotorTimeout();
	}
	
	private class CalibrateThread extends Thread {
		public void run() {
			try {
				liftMotor.enableReverseSoftLimit(false);
				liftMotor.enableForwardSoftLimit(false);
				liftMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
				int counter = 100;
				while (counter-- > 0 && !liftMotor.isRevLimitSwitchClosed()) {
					liftMotor.set(-0.2);
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
				liftMotor.enableForwardSoftLimit(true);
				System.out.println( "Finished Calibrating" );
			}
		}
	}


	public void iterate() {
		if (firstRun) {
			SmartDashboard.putBoolean( "PresetButtonPressed", false );
			//calibrate();
			firstRun = false;
		}
		if( SmartDashboard.getBoolean("Reset") == true ) {
			calibrate();
			SmartDashboard.putBoolean("Reset", false);
		} else if (calibrateThread == null) {
			iterateLift();
		}

		forkgrabber.iterate();

	}

	/** run the lifting code */
	public void iterateLift() {
		
		// Continuous "Auto Calibrate"
		if (liftMotor.isRevLimitSwitchClosed()) {
			liftMotor.setPosition(0);
		}

		//boolean buttonIsPressed = SmartDashboard.getBoolean("PresetButtonPressed");

		double currentJoystick;
		double currentPosition;
		double desired = 0;

		double actualForkOffset = 0;


		currentJoystick = Robot.getRobot().getManipulatorJoystick().getY();
		currentPosition = liftMotor.getPosition();
		SmartDashboard.putNumber("currentPos", currentPosition);
		desired = currentPosition;

		if( Math.abs( currentJoystick ) > RobotMap.DRIVER_JOYSTICK_TOLERANCE ) {
			SmartDashboard.putBoolean( "PresetButtonPressed", false );
		}

		if( SmartDashboard.getBoolean( "PresetButtonPressed" ) ) {
			//get lab view values
			desired = SmartDashboard.getNumber("PresetValue") + SmartDashboard.getNumber("Offset")
					+ SmartDashboard.getNumber("ContainerOffset") + SmartDashboard.getNumber("DropOffset");
			liftMotor.enableControl();
			//applyBrake( false );

		} else {
			//manipulator control
			if( Math.abs( currentJoystick ) > RobotMap.DRIVER_JOYSTICK_TOLERANCE ) {
				if( currentJoystick > 0.0 ) {
					desired = currentPosition + 700;		//// min position should be zero.... max position should be 7810
				} else {
					desired = currentPosition - 700;
				}
			}

			liftMotor.enableControl();
			//applyBrake( false );

		}

		if (desired != lastSetPoint) {
			currentTime = System.currentTimeMillis();
			lastSetPoint = desired;
		}

		// If the speed is low and it has been running for a while, stop moving
		if (( liftMotor.getSpeed() <= RobotMap.DRIVER_JOYSTICK_TOLERANCE ) && ( System.currentTimeMillis() - currentTime >= 5000 )) {
			desired = currentPosition;
			liftMotor.disableControl();
		}

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

		//=================================================================================	TEST CODE TODO



		//System.out.println( "...2...Current Motor: " + currentPosition + "    Desired: " + desired + "      Current Joystick: " + currentJoystick );
		moveTo( desired );


		if( Math.abs( liftMotor.getPosition() - desired ) < 50 ) {
			applyBrake( true );
			liftMotor.set(currentPosition);
		} else {
			applyBrake( false );
		}

		//System.out.println( "Wanted Value: " + desired + "\t    Lift motor position: " + liftMotor.getPosition() );
		Sensors.setAnalogOutput(liftMotor.getEncPosition()/2000);
	}


	public void applyBrake( boolean brakeBoolean) {

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
	
	public int getPosition() {
		return liftMotor.getAnalogInRaw();
	}
	
	/**
	 * for autonomous only
	 */
	public void setPercentVBusMode (boolean vBus) {
		if (vBus) {
			liftMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		}else {
			liftMotor.changeControlMode(CANTalon.ControlMode.Position);
		}
	}
	

	public void moveTo( double desired ) {
		liftMotor.set( desired );
	}



}
