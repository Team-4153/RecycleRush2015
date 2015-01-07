
package org.usfirst.frc.team4153.robot;

import java.nio.IntBuffer;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.hal.PDPJNI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	Talon talon1;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	// This line when deployed seems to get rid of the sticky fault
    	// but prevents robot code from happening
    	//PDPJNI.clearPDPStickyFaults(IntBuffer.wrap(new int[]{0}));

    	talon1 = new Talon(1);
    	try {
    		// CANJaguar testJag = new CANJaguar(1);
    	}catch (Exception e){
    		// This will throw an error because there is no can
    		
    	}
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	//CANJaguar jag1 = new CANJaguar(1);
    	
    	talon1.set(0.25);
    }
    

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	talon1.set(0.);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
