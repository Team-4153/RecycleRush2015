package org.usfirst.frc.team4153.robot;
import edu.wpi.first.wpilibj.smartdashboard.*;

import org.usfirst.frc.team4153.robot.subsystems.Chassis;
import org.usfirst.frc.team4153.robot.subsystems.Forklift;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	Chassis chassis;
	Forklift forklift;
	
	 

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		// This line when deployed seems to get rid of the sticky fault
		// but prevents robot code from happening
		// PDPJNI.clearPDPStickyFaults(IntBuffer.wrap(new int[]{0}));

		

		//Joystick driveStick = new Joystick(RobotMap.DRIVER_JOYSTICK);
		chassis = new Chassis();
		chassis.init();
		
		
		
		forklift = new Forklift();
		

	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		chassis.joystickDrive();	
		
	//	wantedPosition = Sensors.getManipulatorJoystick() + Forklift.liftMotor.getPosition();
		forklift.moveLiftMotor(wantedPosition);
		
		
		
	}

	/**
	 * This function is called periodically during test mode
	 */

	public void testPeriodic() {
		
		 boolean tempb = SmartDashboard.getBoolean("dashButton1");
		 System.out.println(tempb);
		 int tempv = (int) SmartDashboard.getNumber("dashPot1",0);
		 System.out.println(tempv);
		 SmartDashboard.putBoolean("returnBool",SmartDashboard.getBoolean("dashButton1"));

	}

}
