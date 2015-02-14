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
	
	protected int maxSpeed;
	protected int minSpeed = 1000;
	
	protected int counter = 0;
	
	protected long lastTime;
	
	protected double tickPer10ms;
	
	protected long lastTics;

	public void init() {
		// Initialize the Talons with the correct modes, etc.
		//frontRight = new CANTalon(RobotMap.FRONT_RIGHT_TALON);
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
	
		double P=1;
		double I= 0;  //0;.001;
		double D=0;
		double feedforward=.001;
		double ramp=360;
		frontRight.reverseOutput(false);
		frontRight.reverseSensor(false);
		frontLeft.reverseOutput(true);
		frontLeft.reverseSensor(true);
		backRight.reverseOutput(true);
		backRight.reverseSensor(false);
		backLeft.reverseOutput(false);
		backLeft.reverseSensor(true);
		//Front left PID ( 0.25, 0.0020, 0, 0.10, 00, -30, 0);
		
		frontRight.setPID( 0.25, 0.0020, 0, 0.10, 00, -30, 0);
		frontLeft.setPID( 0.25, 0.0020, 0, 0.10, 00, -30, 0);
		backRight.setPID( 0.25, 0.0020, 0, 0.10, 00, -30, 0);
		backLeft.setPID( 0.25, 0.0020, 0, 0.10, 00, -30, 0);
		
//		frontRight.ClearIaccum();
//		frontLeft.ClearIaccum();
//		backRight.ClearIaccum();
//		backLeft.ClearIaccum();
		
//		frontRight.setProfile(0);
//		frontLeft.setProfile(0);
//		backRight.setProfile(0);
//		backLeft.setProfile(0);

		drive = new RobotDrive( frontLeft, backLeft, frontRight, backRight );
		
		//drive.setSafetyEnabled(false);												//disable safety restrictions to boost performance
		//System.out.println("chassis init");
		lastTics = frontRight.getEncPosition();
		lastTime = System.currentTimeMillis();
	}

	public void joystickDriveIterate() {
//		double gyroAngle = Sensors.getGyroAngle();
//		Joystick driverJoystick = Sensors.getDriverJoystick();
//		drive.mecanumDrive_Polar(driverJoystick.getMagnitude(),
//			driverJoystick.getDirectionDegrees(), gyroAngle);		
//		//drive.mecanumDrive_Cartesian(.1, .1, 0, 0);
//		//drive.arcadeDrive(driverJoystick);
		
		//frontRight.setPID( 0.25, 0.00050, 0); // , 00, 0, 0, 0); 
		/*frontRight.changeControlMode(ControlMode.Speed);
		frontRight.set(150);
		
		frontLeft.changeControlMode(ControlMode.Speed);
		frontLeft.set(150);
		
		backRight.changeControlMode(ControlMode.Speed);
		backRight.set(150);
		
		backLeft.changeControlMode(ControlMode.Speed);
		backLeft.set(150);
		double currentSpeed = frontRight.getSpeed();
		if ( currentSpeed < minSpeed ) {
			minSpeed = (int)currentSpeed;
		}
		else if ( currentSpeed > maxSpeed ) {
			maxSpeed = (int)currentSpeed;
		}
		if ( ( counter++ % 200 ) == 0 ) {
			maxSpeed = 0 ;
			minSpeed = 1000;
		}
		long timeNow = System.currentTimeMillis();
		long deltaT = timeNow - lastTime;
		long ticsNow = frontRight.getEncPosition();
		long deltaPosition = lastTics - ticsNow;
		tickPer10ms =  (tickPer10ms * 9 + deltaPosition / deltaT *10)/10;
		System.out.println( "frontRight encoder " + currentSpeed + " " + minSpeed + " " + maxSpeed + " " + frontRight.getSetpoint() + " " + (int) tickPer10ms);
		lastTics = ticsNow;
		lastTime = timeNow;*/
		Joystick joystick = Sensors.getDriverJoystick();
		//System.out.println("Joystick xy: " + joystick.getX()+", " + joystick.getY());
		drive.setMaxOutput(1200);
		//System.out.println(frontRight.getSetpoint());
		
		
		drive.setSafetyEnabled(false);
		
		drive.arcadeDrive(joystick);
		
		//drive(joystick.getX(), joystick.getY(), joystick.getTwist());
		drive.mecanumDrive_Cartesian(joystick.getX(), joystick.getY(), joystick.getTwist(), Sensors.getGyroAngle());
		
		if(joystick.getRawButton(2) == true) {
			Sensors.gyroReset();
			System.out.println("Gyro Reset Succesfull");
		}
	}

	public void drive(double x, double y, double rotation) {
	
		drive.mecanumDrive_Cartesian(x, y, rotation, Sensors.getGyroAngle());
	}

	@Override
	public void iterate() {
		joystickDriveIterate();		
	}

	@Override
	public void reset() {
		// TODO Auto-generated method stub
		
	}

}
