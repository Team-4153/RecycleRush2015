package org.usfirst.frc.team4153.robot.subsystems;

import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

public class Chassis implements Subsystem {
	//The motor controllers for each wheel
	protected CANTalon frontRight;
	protected CANTalon frontLeft;
	protected CANTalon backRight;
	protected CANTalon backLeft;

	protected RobotDrive drive;

	private boolean isMechanum = true;

	public void init() {
		frontRight = new CANTalon(RobotMap.FRONT_RIGHT_TALON, 1);
		frontLeft = new CANTalon(RobotMap.FRONT_LEFT_TALON);
		backRight = new CANTalon(RobotMap.BACK_RIGHT_TALON);
		backLeft = new CANTalon(RobotMap.BACK_LEFT_TALON);

		frontRight.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		frontLeft.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		backRight.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		backLeft.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

		frontRight.changeControlMode(CANTalon.ControlMode.Speed);
		frontLeft.changeControlMode(CANTalon.ControlMode.Speed);
		backRight.changeControlMode(CANTalon.ControlMode.Speed);
		backLeft.changeControlMode(CANTalon.ControlMode.Speed);

		//Left and right are reversed from each other
		//Front motors have reversed sensors (because of belt drive)
		frontRight.reverseOutput(false);
		frontRight.reverseSensor(false);
		frontLeft.reverseOutput(true);
		frontLeft.reverseSensor(true);
		backRight.reverseOutput(true);
		backRight.reverseSensor(false);
		backLeft.reverseOutput(false);
		backLeft.reverseSensor(true);

		frontRight.setPID(RobotMap.DRIVER_P, RobotMap.DRIVER_I,
				RobotMap.DRIVER_D, RobotMap.DRIVER_FEED_FORWARD, 00,
				RobotMap.DRIVER_RAMP, 0);
		frontLeft.setPID(RobotMap.DRIVER_P, RobotMap.DRIVER_I,
				RobotMap.DRIVER_D, RobotMap.DRIVER_FEED_FORWARD, 00,
				RobotMap.DRIVER_RAMP, 0);
		backRight.setPID(RobotMap.DRIVER_P, RobotMap.DRIVER_I,
				RobotMap.DRIVER_D, RobotMap.DRIVER_FEED_FORWARD, 00,
				RobotMap.DRIVER_RAMP, 0);
		backLeft.setPID(RobotMap.DRIVER_P, RobotMap.DRIVER_I,
				RobotMap.DRIVER_D, RobotMap.DRIVER_FEED_FORWARD, 00,
				RobotMap.DRIVER_RAMP, 0);

		//Creates the drivetrain object with the 4 motors
		drive = new RobotDrive(frontLeft, backLeft, frontRight, backRight);

		drive.setSafetyEnabled(false); // disable safety restrictions to boost
		// performance

		drive.setMaxOutput( RobotMap.DRIVER_MAX_OUTPUT );	

	}

	/**
	 * Called periodically during Teleop
	 */
	public void iterate() {

		Joystick joystick = Sensors.getDriverJoystick();
		//System.out.println("Joystick xy: " + joystick.getX()+", " + joystick.getY());


		//1300 current number.... if debug put actual value to test


		if(joystick.getRawButton(2) == true) {	// check if the gyro needs reseting
			Sensors.gyroReset();
			System.out.println("Gyro Reset Succesfull");
		}


		//---------------------------------------check for mechanum

		if( joystick.getRawButton( 8 ) ) {
			isMechanum = true;
		}
		/**if( joystick.getRawButton( 7 ) ) { //Arcade drive mode disabled
			isMechanum = false;
		}*/

		//================================================

		//Ask whoever wrote this bit. For now, it's probably a good idea to not press button 3, it slows the robot down a whole bunch
		boolean robotDrive = joystick.getRawButton(3);
		double throttle = joystick.getThrottle()*RobotMap.ROBOT_CONTROL_MODIFIER;
		if (robotDrive) {
			throttle*=RobotMap.ROBOT_CONTROL_MODIFIER;
		}

		//Drives with the appropriate method (arcade (disabled) or mecanum-cartesian) if the joystick is out of the deadzone
		if( Math.abs(joystick.getX() ) > RobotMap.DRIVER_JOYSTICK_TOLERANCE || 
				Math.abs( joystick.getY() ) > RobotMap.DRIVER_JOYSTICK_TOLERANCE || 
				Math.abs( joystick.getZ() ) > RobotMap.DRIVER_JOYSTICK_TOLERANCE )  {  		//all values have a range from -1 to 1
			if( isMechanum ) {
				// for robot drive, it assumes that the gyro angle is 0
				drive.mecanumDrive_Cartesian(joystick.getX()*throttle, joystick.getY()*throttle, joystick.getTwist()*RobotMap.DRIVER_JOYSTICK_TWIST_MODIFIER, Sensors.getGyroAngle());
			} else {
				drive.arcadeDrive(joystick);
			}

			//System.out.println( "Drive Joystick... X: " + joystick.getX() + "   Y: " + joystick.getY() + "  Z: " + joystick.getZ() );
		} else {

			if( isMechanum ) {
				drive.mecanumDrive_Cartesian( 0.0 , 0.0, 0.0, 0.0 );
			} else {
				drive.arcadeDrive(joystick);
			}
		}



	}

	/** 
	 * Orders the drive to speeds x and y
	 * @param x [-1.0, 1.0]
	 * @param y [-1.0, 1.0]
	 */
	public void autoDrive(double x, double y, double turn) {
		drive.mecanumDrive_Cartesian(x, y, turn, Sensors.getGyroAngle());
	}

	@Override
	public void reset() {
		// TODO Auto-generated method stub

	}

}
