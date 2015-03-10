package org.usfirst.frc.team4153.robot.subsystems;

import org.usfirst.frc.team4153.robot.Robot;
import org.usfirst.frc.team4153.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDHandler implements Subsystem {
	private boolean blueAlliance = false;
	private DigitalOutput alliancePort;
	private DigitalOutput armsOpenPort;
	private DigitalOutput stallPort;
	private DigitalOutput enablePort;

	@Override
	public void init() {
		Alliance alliance = DriverStation.getInstance().getAlliance();
		blueAlliance = alliance.equals(Alliance.Blue);
		
		alliancePort = new DigitalOutput(RobotMap.ARDUINO_ALLIANCE_COLOR_PORT);
		armsOpenPort = new DigitalOutput(RobotMap.ARDUINO_ARMS_OPEN_PORT);
		stallPort    = new DigitalOutput(RobotMap.ARDUINO_ARMS_STALLED_PORT);
		enablePort   = new DigitalOutput(RobotMap.ARDUINO_ENABLE_PORT);
	}

	@Override
	public void iterate() {
		alliancePort.set(blueAlliance);
		armsOpenPort.set(SmartDashboard.getBoolean( "OpenFork", true ));
		stallPort.set(Robot.getRobot().getForklift().forkgrabber.checkStalling());
		enablePort.set(true);
	}
	
	/**
	 * called while the robot is disabled to disable the arduino
	 */
	public void disable() {
		enablePort.set(false);
	}

	@Override
	public void reset() {
		// TODO Auto-generated method stub

	}

}
