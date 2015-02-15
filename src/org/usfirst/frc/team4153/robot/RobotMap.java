package org.usfirst.frc.team4153.robot;

// RobotMap will show ports and correct circuitry set-up and constants for computer

public abstract class RobotMap {

	// Analog Ports
	public static final int GYRO = 0;
	public static final int DEBUG = 11;

	// Digital Ports
	

	// Joysticks
	public static final int DRIVER_JOYSTICK = 0;
	public static final int MANIPULATOR_JOYSTICK = 1;
	public static final double DRIVER_JOYSTICK_TOLERANCE = .2;

	// Motor ports
	public static final int FRONT_RIGHT_TALON = 2;
	public static final int FRONT_LEFT_TALON = 4;
	public static final int BACK_RIGHT_TALON = 1;
	public static final int BACK_LEFT_TALON = 3;
	public static final int LIFT_MOTOR = 6;
	public static final int LIFT_MOTOR2 = 8;    	
	public static final int FORK_MOTOR = 7;
	public static final int BRAKE_MOTOR = 5;

	// Encoder
	public static final int ENCODER_FACTOR = 4;
	public static final int CLICKS_PER_ROTATION = 1440;			//encoder factor * 360
	
	//PID Constants
	public static final double DRIVER_P = 0.25;
	public static final double DRIVER_I = 0.002;
	public static final double DRIVER_D = 0.0;
	public static final double DRIVER_FEED_FORWARD = 0.10;
	public static final int DRIVER_RAMP = -30;
	public static final double DRIVER_MAX_OUTPUT = 1300;			//not part of PID
	
	public static final double LIFT_MOTOR_P = 0.25;
	public static final double LIFT_MOTOR_I = .00015;
	public static final double LIFT_MOTOR_D = 0.0;
	public static final double LIFT_MOTOR_FEED_FORWARD = .000;
	public static final int LIFT_MOTOR_RAMP = 0;
	
	public static final double BRAKE_MOTOR_P = 0.1;
	public static final double BRAKE_MOTOR_I = 0.001;
	public static final int BRAKE_MOTOR_D = 1;
	public static final double BRAKE_MOTOR_FEED_FORWARD = 0.001;
	public static final int BRAKE_MOTOR_RAMP = 36;
	
	
	
	
	

}
