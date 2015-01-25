package org.usfirst.frc.team4153.robot.subsystems;
import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.CANJaguar.ControlMode;
import edu.wpi.first.wpilibj.CANTalon;

public class Forklift extends Subsystem {
	
	private CANTalon liftMotor;
	
	public Forklift() {
		
		liftMotor = new CANTalon( RobotMap.LIFT_MOTOR );
		liftMotor.changeControlMode( CANTalon.ControlMode.PercentVbus);	
		liftMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		liftMotor.setPosition(0);
		liftMotor.ClearIaccum();
	
		
		

	}
	public void moveLiftMotor( double wantedRotations) {
		double currentRotations;
		currentRotations = Math.abs(liftMotor.getPosition() / 1440);
		
		//System.out.println( currentRotations );
		
		
		
		if( currentRotations < wantedRotations ) { 
			liftMotor.set( 0.19 );
		} else {
			liftMotor.set( 0.0 );
		}
		
		
		
		
		
	}
}
