package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.CANJaguar.ControlMode;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Forklift extends Subsystem {
	
	private CANTalon liftMotor;
	private static Joystick manipulatorJoystick;
	
	public Forklift() {
		manipulatorJoystick = new Joystick(RobotMap.MANIPULATOR_JOYSTICK);
		liftMotor = new CANTalon( RobotMap.LIFT_MOTOR );
		liftMotor.changeControlMode( CANTalon.ControlMode.Position);	
		liftMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		liftMotor.setPosition(0);
		liftMotor.setPID(.001,0.001,1);
		liftMotor.setCloseLoopRampRate(0.05);
		liftMotor.ClearIaccum();
		
		
	
	}
	public void moveLiftMotor( double wantedPosition) {
		System.out.println( "Wanted Value: " + wantedPosition + "\t    Lift motor position: " + liftMotor.getPosition() );
		liftMotor.set(1500);
		
	}
	
	public double findWantedPosition() {
		//boolean buttonIsPressed = SmartDashboard.getBoolean("PresetButtonPressed");
		double x = 0.0;
		/*																							//actual code to be used
		if( Math.abs(manipulatorJoystick.getY()) < 0.05 ) {
			if( buttonIsPressed ) {
				x = SmartDashboard.getNumber("PresetValue");
			} else {
				x = liftMotor.getPosition();
			}
			
		} else {
			x = ( manipulatorJoystick.getY() * RobotMap.CLICKS_PER_ROTATION * 3 ) + liftMotor.getPosition();
			SmartDashboard.putBoolean("PresetButtonPressed", false);
		}
		*/
		
		x = ( manipulatorJoystick.getY() * RobotMap.CLICKS_PER_ROTATION * 3 ) + liftMotor.getPosition();  //test code 
		
		
		return x;
	}
}
