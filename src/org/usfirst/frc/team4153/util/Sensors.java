package org.usfirst.frc.team4153.util;

import org.usfirst.frc.team4153.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Sensors {

	private static Gyro gyro;
	private static AnalogInput debug;
	private static Joystick driverJoystick;
	private static Encoder encoder;
	
	@Deprecated
	public static void init() {						//can delete if not wanted.... doesn't check every time
		
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



	public static double getDebugVoltage() {
		if (debug == null) {
			for (int i = 0; i < 128; i++) {
				try {
					debug = new AnalogInput(i);
					SmartDashboard.putNumber("DebugChannel", i);
				} catch (Exception e) {
					System.out.println("Not channel " + i);
				}
			}

		}
		return debug.getVoltage(); // send the angle back
	}

	public void reset() {
		getGyro().reset(); 			//if disabled 
	}
	
	
}
