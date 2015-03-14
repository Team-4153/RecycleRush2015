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
	
	// Joystick constants
	public static final double DRIVER_JOYSTICK_TOLERANCE = .1;
	public static final double DRIVER_JOYSTICK_TWIST_MODIFIER = 0.35;
	public static final double DRIVER_MAX_OUTPUT = 1450;					//not part of PID
	
	public static final double ROBOT_CONTROL_MODIFIER = 0.5;

	// Motor ports
	public static final int FRONT_RIGHT_TALON = 2;
	public static final int FRONT_LEFT_TALON = 4;
	public static final int BACK_RIGHT_TALON = 1;
	public static final int BACK_LEFT_TALON = 3;
	public static final int LIFT_MOTOR = 6;
	public static final int LIFT_MOTOR2 = 8;    	
	public static final int FORK_MOTOR = 7;
	public static final int BRAKE_MOTOR = 5;
	public static final int SPINNER_RIGHT = 1;
	public static final int SPINNER_LEFT = 2;

	// Encoder
	public static final int ENCODER_FACTOR = 4;
	public static final int CLICKS_PER_ROTATION = 1440;			//encoder factor * 360
	
	//PID Constants
	public static final double DRIVER_P = 0.25;
	public static final double DRIVER_I = 0.002;
	public static final double DRIVER_D = 0.0;
	public static final double DRIVER_FEED_FORWARD = 0.10;
	public static final int DRIVER_RAMP = -30;
	
	
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
	public static final int FORK_POSITION_TOLERANCE = 5;
	
	// Spinner constants: both should be positive
	public static final double SPIN_FORWARD_SPEED = 0.4;
	public static final double SPIN_REVERSE_SPEED = 0.3;
	
	// Spinner switch states
	public static final int SPIN_FORWARD_STATE = 0;
	public static final int SPIN_OFF_STATE = 1;
	public static final int SPIN_REVERSE_STATE = 2;
	// the spinner will run if it is in off state while this button is pressed
	public static final int JOYSTICK_SPIN_BUTTON = 1;
	
	// Arduino digital I/O
	// All values are largely arbitrary
	public static final int ARDUINO_ALLIANCE_COLOR_PORT = 3;
	public static final int ARDUINO_ARMS_OPEN_PORT = 4;
	public static final int ARDUINO_ARMS_STALLED_PORT = 5;
	public static final int ARDUINO_ENABLE_PORT = 6;
	
	
	
	// Auto modes
	public static final int DRIVE_FORWARD_MODE = 0;
	public static final int PICK_TOTE_AND_DRIVE = 1;
	public static final int PICK_TOTE_AND_PUSH_BIN = 2;
	public static final int DO_NOTHING = 3;
	
	
	

}
