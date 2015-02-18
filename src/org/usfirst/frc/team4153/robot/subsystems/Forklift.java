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
	private final int FORWARD_SOFT_LIMIT = 11000;
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
		//liftMotor.changeControlMode( CANTalon.ControlMode.Position);
		liftMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		liftMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		liftMotor.setPosition(0);  

		liftMotor.setPID(RobotMap.LIFT_MOTOR_P, RobotMap.LIFT_MOTOR_I, RobotMap.LIFT_MOTOR_D, RobotMap.LIFT_MOTOR_FEED_FORWARD, 0, RobotMap.LIFT_MOTOR_RAMP, 0);
		liftMotor.setProfile(0);
		liftMotor.ClearIaccum();
		liftMotor.reverseSensor(true);
		liftMotor.reverseOutput( false );
		liftMotor.enableBrakeMode( true );
		liftMotor.setReverseSoftLimit( 100 );
		liftMotor.setForwardSoftLimit(FORWARD_SOFT_LIMIT); 			/// TODO Check and change
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


		// calibrate();			//taken out temporarily
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
		forkgrabber.close();
	}

	public void open() {
		forkgrabber.open();
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
					//System.out.println("Calibrating");
					try {
						sleep(10);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}


			} finally {
				liftMotor.set(0);
				//liftMotor.changeControlMode(CANTalon.ControlMode.Position);
				liftMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
				liftMotor.setPosition(0);
				//==================================================
				try {
					sleep( 200 );
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				//==================================================
				calibrateThread = null;
				liftMotor.enableReverseSoftLimit(true);
				liftMotor.enableForwardSoftLimit(true);
				System.out.println( "Finished Calibrating! ... Encoder Value: " + liftMotor.getPosition() );
				SmartDashboard.putBoolean( "PresetButtonPressed", false );
			}
		}
	}


	public void iterate() {
		boolean temporary = false;
		try {
			temporary = SmartDashboard.getBoolean("Reset");
		} catch (Exception e) {
			e.printStackTrace();			
		}
		if (firstRun) {
			SmartDashboard.putBoolean( "PresetButtonPressed", false );
			//calibrate();
			firstRun = false;
		}
		if( temporary ) {
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


		double currentJoystick;
		double currentPosition;
		double desired = 0;

		//double actualForkOffset = 0;


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
			//liftMotor.enableControl();
			//applyBrake( false );

		} else {
			//manipulator control
			if( Math.abs( currentJoystick ) > RobotMap.DRIVER_JOYSTICK_TOLERANCE ) {
				if( currentJoystick > 0.0 ) {
					desired = currentPosition + 1100;		//// min position should be zero.... max position should be 7810
				} else {
					desired = currentPosition - 1100;
				}
			}

			//liftMotor.enableControl();
			//applyBrake( false );
		}


		if (desired > FORWARD_SOFT_LIMIT - 100)
		{
			desired = FORWARD_SOFT_LIMIT - 100;
		}



		if (desired != lastSetPoint) {
			currentTime = System.currentTimeMillis();
			lastSetPoint = desired;
		}

		// If the speed is low and it has been running for a while, stop moving

		//System.out.println(desired);

		if( Math.abs( liftMotor.getPosition() - desired ) < 200 ) {
			moveTo(currentPosition);
			SmartDashboard.putBoolean( "PresetButtonPressed",false);
			//liftMotor.setPosition(currentPosition);
		} else {
			moveTo(desired);
		}

		//		if (( liftMotor.getSpeed() <= RobotMap.DRIVER_JOYSTICK_TOLERANCE ) && ( System.currentTimeMillis() - currentTime >= 5000 )) {
		//			applyBrake(true);
		//			desired = currentPosition;
		//			//liftMotor.disableControl();
		//			
		//		}
		//System.out.println( "Wanted Value: " + desired + "\t    Lift motor position: " + liftMotor.getPosition() );
		Sensors.setAnalogOutput(-liftMotor.getEncPosition()/2000);

	}
	
	public void setZero () {
		liftMotor.setPosition(0);
	}


	protected void applyBrake( boolean brakeBoolean) {

		if ( brakeBoolean ) {
			brakeMotor.set(-0.3);
			liftMotor.disableControl();		
		}
		else {
			brakeMotor.set(0.3);
			liftMotor.enableControl();			
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
			//liftMotor.changeControlMode(CANTalon.ControlMode.Position);
			liftMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		}
	}

	int count = 0;
	public void moveTo( double desired ) {
		count ++;
		if (count % 10 == 0) {
			System.out.println("Wanted: " + desired+ ", Current: " + liftMotor.getPosition());
		}
		// should only try to move if not calibrating
		if (calibrateThread == null) {
			if (Math.abs(liftMotor.getPosition() - desired) > 3000) {
				applyBrake(false);
				if (desired > liftMotor.getPosition()) {
					liftMotor.set(0.6);
				}
				else {
					liftMotor.set(-0.3);
				}
			}
			else if (Math.abs(liftMotor.getPosition() - desired) > 1000) {
				applyBrake(false);
				if (desired > liftMotor.getPosition()) {
					liftMotor.set(0.4);
				}
				else {
					liftMotor.set(-0.2);
				}
			}
			else if (Math.abs(liftMotor.getPosition() - desired) > 500) {
				applyBrake(false);
				if (desired > liftMotor.getPosition()) {
					liftMotor.set(0.3);
				}
				else {
					liftMotor.set(-0.15);
				}
			}
			else if (Math.abs(liftMotor.getPosition() - desired) > 50){
				applyBrake(false);
				if (desired > liftMotor.getPosition()) {
					liftMotor.set(0.25);
				}
				else {
					liftMotor.set(-0.15);
				}
			}else {
				applyBrake(true);
			}
		}
	}



}
