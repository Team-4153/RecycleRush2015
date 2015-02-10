package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.CANJaguar.ControlMode;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Forklift extends Subsystem {
	
	private CANTalon liftMotor, forkMotor, brakeMotor;
	private static Joystick manipulatorJoystick;
	
	//Difference between lift motor position and desired position (in encoder value) that is considered "close enough" to the desired position
	private final double BRAKE_TOLERANCE = 0.01; //Set later
	
	public void init() {
		manipulatorJoystick = new Joystick(RobotMap.MANIPULATOR_JOYSTICK);
		liftMotor = new CANTalon( RobotMap.LIFT_MOTOR );
		liftMotor.changeControlMode( CANTalon.ControlMode.Position);	
		liftMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		liftMotor.setPosition(0);  
		liftMotor.setPID(0.1, .001, 1, .001, 100, 36, 0);   		//magical numbers...manual
		liftMotor.setProfile(0);
		liftMotor.ClearIaccum();
		liftMotor.reverseSensor(true);
		
		forkMotor = new CANTalon(RobotMap.FORK_MOTOR);
		forkMotor.changeControlMode(CANTalon.ControlMode.Position);
		forkMotor.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		forkMotor.setPosition(0);
		forkMotor.setPID(0.1, 0.001, 1, 0.001, 100, 36, 0);
		forkMotor.setProfile(0);
		forkMotor.ClearIaccum();
		
		brakeMotor = new CANTalon(RobotMap.BRAKE_MOTOR);
		brakeMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		brakeMotor.setPosition(0);
		brakeMotor.setPID(0.1, 0.001, 1, 0.001, 100, 36, 0);
		brakeMotor.setProfile(0);
		brakeMotor.ClearIaccum();
	}
	
	public void reset() {
		liftMotor.setPosition( 0.0 );
		forkMotor.setPosition(0.0);
	}
	
	public void moveLiftMotor( double wantedPosition) {
		System.out.println( "Wanted Value: " + wantedPosition + "\t    Lift motor position: " + liftMotor.getPosition() );
		liftMotor.set(1500);
		//liftMotor.set(wantedPosition);
		
	}
	
	public void iterateLiftMotor() {
		//boolean buttonIsPressed = SmartDashboard.getBoolean("PresetButtonPressed");
		double desired = 0.0;
		/*																						//actual code to be used
		if( Math.abs(manipulatorJoystick.getY()) < 0.05 ) {
			if( buttonIsPressed ) {
				desired = SmartDashboard.getNumber("PresetValue") + SmartDashboard.getNumber("OffsetValue") + SmartDashboard.getNumber("ContainerOffset");
			} else {
				desired = liftMotor.getPosition();
			}
			
		} else {
			desired = ( manipulatorJoystick.getY() * RobotMap.CLICKS_PER_ROTATION * 3 ) + liftMotor.getPosition();
			SmartDashboard.putBoolean("PresetButtonPressed", false);
		}
		*/
		
		
		
		/*
		desired = ( manipulatorJoystick.getY() * RobotMap.CLICKS_PER_ROTATION * 3 ) + liftMotor.getPosition();  //test code 
		
		System.out.println( "Wanted Value: " + desired + "\t    Lift motor position: " + liftMotor.getPosition() );
		if (Math.abs(liftMotor.get() - desired) <= BRAKE_TOLERANCE) {
			liftMotor.disableControl();
		    iterateBrake(true);
		}
		else {
			liftMotor.enableControl();
			liftMotor.set(desired);	
		    iterateBrake(false);
		}
		*/
		iterateBrake(false);
		
	}
	
	public void iterateForkMotor() {
		//forkMotor.set(SmartDashboard.getNumber("ForkPreset"));
	}
	
	public void iterateBrake( boolean brakeBoolean) {
		
		if (brakeBoolean) {											//Code to activate brake
			if( brakeMotor.isRevLimitSwitchClosed() ) {
				brakeMotor.set( 0.0 );
			} else {
				brakeMotor.set( -0.2 );
			}
			
		} else {													//Code to release brake
			if( brakeMotor.isFwdLimitSwitchClosed() ) {
				brakeMotor.set( 0.0 );
			} else {
				brakeMotor.set( 0.3 );
			}
		}
	}
}
