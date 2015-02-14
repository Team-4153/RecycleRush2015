package org.usfirst.frc.team4153.robot.subsystems;

import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

public class Chassis implements Subsystem {
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
	
		
		frontRight.reverseOutput(false);
		frontRight.reverseSensor(false);
		frontLeft.reverseOutput(true);
		frontLeft.reverseSensor(true);
		backRight.reverseOutput(true);
		backRight.reverseSensor(false);
		backLeft.reverseOutput(false);
		backLeft.reverseSensor(true);
		
		
		frontRight.setPID( RobotMap.DRIVER_P, RobotMap.DRIVER_I, RobotMap.DRIVER_D, RobotMap.DRIVER_FEED_FORWARD, 00, RobotMap.DRIVER_RAMP, 0);
		frontLeft.setPID(  RobotMap.DRIVER_P, RobotMap.DRIVER_I, RobotMap.DRIVER_D, RobotMap.DRIVER_FEED_FORWARD, 00, RobotMap.DRIVER_RAMP, 0);
		backRight.setPID(  RobotMap.DRIVER_P, RobotMap.DRIVER_I, RobotMap.DRIVER_D, RobotMap.DRIVER_FEED_FORWARD, 00, RobotMap.DRIVER_RAMP, 0);
		backLeft.setPID(   RobotMap.DRIVER_P, RobotMap.DRIVER_I, RobotMap.DRIVER_D, RobotMap.DRIVER_FEED_FORWARD, 00, RobotMap.DRIVER_RAMP, 0);


		drive = new RobotDrive( frontLeft, backLeft, frontRight, backRight );
		
		drive.setSafetyEnabled(false);												//disable safety restrictions to boost performance
		
	}

	public void iterate() {

		Joystick joystick = Sensors.getDriverJoystick();
		//System.out.println("Joystick xy: " + joystick.getX()+", " + joystick.getY());
		
		
		drive.setMaxOutput( RobotMap.DRIVER_MAX_OUTPUT );			//1300 current number.... if debug put actual value to test
		
		
		if(joystick.getRawButton(2) == true) {				//reset gyro
			Sensors.gyroReset();
			System.out.println("Gyro Reset Succesfull");
		}
		
		
		if( joystick.getRawButton( 8 ) ) {
			isMechanum = true;
		}
		if( joystick.getRawButton( 7 ) ) {
			isMechanum = false;
		}
		
		
			
		if( isMechanum ) {
			drive.mecanumDrive_Cartesian(joystick.getX(), joystick.getY(), joystick.getTwist(), Sensors.getGyroAngle());
		} else {
			drive.arcadeDrive(joystick);
		}
	
		
	}

	

	@Override
	public void reset() {
		// TODO Auto-generated method stub
		
	}

}
