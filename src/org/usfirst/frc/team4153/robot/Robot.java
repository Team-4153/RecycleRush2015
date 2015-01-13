
package org.usfirst.frc.team4153.robot;

import org.usfirst.frc.team4153.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	Chassis chassis;
	
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	// This line when deployed seems to get rid of the sticky fault
    	// but prevents robot code from happening
    	//PDPJNI.clearPDPStickyFaults(IntBuffer.wrap(new int[]{0}));
    	
    	Joystick driveStick = new Joystick(0);
    	chassis = new Chassis( );
    	chassis.init();
    	
    	
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	//CANJaguar jag1 = new CANJaguar(1);
    
    	
    }
    

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	chassis.drive( 0.25, 0.25 );
    	
    }
    
}
