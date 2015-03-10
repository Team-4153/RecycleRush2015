package org.usfirst.frc.team4153.robot.subsystems;

import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Spinners implements Subsystem {

	private Talon rightSpinner;
	private Talon leftSpinner;
	
	@Override
	public void init() {
		rightSpinner = new Talon(RobotMap.SPINNER_RIGHT);
		leftSpinner  = new Talon(RobotMap.SPINNER_LEFT);


		rightSpinner.setSafetyEnabled(false);
		leftSpinner.setSafetyEnabled(false);
	}

	@Override
	public void iterate() {
		// Now check for spinner status
				int spinnerState = (int) SmartDashboard.getNumber("Intake");
				switch (spinnerState) {
				case (RobotMap.SPIN_FORWARD_STATE):
					forward();
				break;
				case (RobotMap.SPIN_OFF_STATE):{
					// The spinner will be off unless the driver trigger is pressed
					if (Sensors.getDriverJoystick().getRawButton(RobotMap.JOYSTICK_SPIN_BUTTON)) {
						reverse();
					} else {
						stopSpinners();
					}
				}
				break;
				case (RobotMap.SPIN_REVERSE_STATE):
					reverse();
				break;
				}
	}

	@Override
	public void reset() {
		// TODO Auto-generated method stub

	}
	
	public void forward() {
		rightSpinner.set(RobotMap.SPIN_FORWARD_SPEED);
		leftSpinner.set(-RobotMap.SPIN_FORWARD_SPEED);
	}

	/**
	 * controls the spinning motors on the end of the arms: reverse means in
	 */
	public void reverse() {
		rightSpinner.set(-RobotMap.SPIN_REVERSE_SPEED);
		leftSpinner.set(RobotMap.SPIN_REVERSE_SPEED);
	}

	/**
	 * stop the motors from spinning on the ends of the arms
	 */
	public void stopSpinners() {
		rightSpinner.set(0);
		leftSpinner.set(0);
	}

	

}
