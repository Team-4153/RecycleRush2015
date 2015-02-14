package org.usfirst.frc.team4153.util;

import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.robot.subsystems.Subsystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Sensors implements Subsystem {

	private static Gyro gyro;
	private static Joystick driverJoystick;
	private static Encoder encoder;
	
	 {						//can delete if not wanted.... doesn't check every time
		getGyro();
		// This method should not exist: it does not make sense to have to initialize a bunch of static objects
		// I'll leave it here as a warning but it should not be used
		
	}
	 
	public static Encoder getEncoder() {
		if (encoder == null) {
			encoder = new Encoder( RobotMap.ENCODER1, RobotMap.ENCODER0);
		}
		return encoder;
	}

	public static Joystick getDriverJoystick() {
		if (driverJoystick == null) {
			driverJoystick = new Joystick( RobotMap.DRIVER_JOYSTICK);
		}
		return driverJoystick;
	}
	
	public static Gyro getGyro() {
		if (gyro == null) {
			gyro = new Gyro( RobotMap.GYRO);							//if an object is constructed
			gyro.reset();
			System.out.println("Gyro Initalized");
		}
		return gyro;
	}
	
	

	public static double getGyroAngle() {
		
		Joystick driverJoystick = Sensors.getDriverJoystick();

		if( driverJoystick.getRawButton(2) ) {
			gyro.reset();
		}
		return gyro.getAngle(); // send the angle back
		
	}




	public void reset() {
		getGyro().reset(); 			//if disabled 
	}

	public static void gyroReset() {
		gyro.reset();
	}

	@Override
	public void init() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void iterate() {
		// TODO Auto-generated method stub
		
	}
	
	
}
