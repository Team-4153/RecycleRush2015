package org.usfirst.frc.team4153.robot.subsystems;

import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

public class Chassis extends Subsystem {
	CANTalon frontRight;
	CANTalon frontLeft;
	CANTalon backRight;
	CANTalon backLeft;
	RobotDrive drive;

	public void init() {
		frontRight = new CANTalon(RobotMap.FRONT_RIGHT_TALON);
		frontLeft = new CANTalon(RobotMap.FRONT_LEFT_TALON);
		backRight = new CANTalon(RobotMap.BACK_RIGHT_TALON);
		backLeft = new CANTalon(RobotMap.BACK_LEFT_TALON);

		drive = new RobotDrive(frontRight, frontLeft, backRight, backLeft);
		drive.setSafetyEnabled(false);												//disable safety restrictions to boost performance
	}

	public void joystickDrive() {
		double gyroAngle = Sensors.getGyroAngle();
		Joystick driverJoystick = Sensors.getDriverJoystick();
		drive.mecanumDrive_Polar(driverJoystick.getMagnitude(),
				driverJoystick.getDirectionDegrees(), gyroAngle);		
		System.out.println( "New Encoder test value: " + frontRight.getPosition() / 8192 );
		
	}

	public void drive(double x, double y) {
		drive.mecanumDrive_Cartesian(x, y, 0, 0);
	}

}
