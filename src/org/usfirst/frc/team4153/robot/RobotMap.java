package org.usfirst.frc.team4153.robot;

// RobotMap will show ports and correct circuitry set-up and constants for computer

public abstract class RobotMap {

	// Analog Ports
	public static final int GYRO = 0;
	public static final int DEBUG = 11;

	// Digital Ports
	public static final int ENCODER0 = 0;
	public static final int ENCODER1 = 1;

	// Joysticks
	public static final int DRIVER_JOYSTICK = 0;
	public static final int MANIPULATOR_JOYSTICK = 1;

	// Motor ports
	public static final int FRONT_RIGHT_TALON = 2;
	public static final int FRONT_LEFT_TALON = 4;
	public static final int BACK_RIGHT_TALON = 1;
	public static final int BACK_LEFT_TALON = 3;
	public static final int LIFT_MOTOR = 6;
	public static final int LIFT_MOTOR2 = 8;    	///!!!!!????? CHANGE
	public static final int FORK_MOTOR = 7;
	public static final int BRAKE_MOTOR = 5;

	// Encoder
	public static final int ENCODER_FACTOR = 4;
	public static final int CLICKS_PER_ROTATION = 1440;			//encoder factor * 360

}
