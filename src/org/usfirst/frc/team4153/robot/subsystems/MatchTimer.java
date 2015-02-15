package org.usfirst.frc.team4153.robot.subsystems;

import org.usfirst.frc.team4153.robot.Robot;

public class MatchTimer implements Subsystem {
	
	private double startTime = 0;		//time to apply the break at the the very end
	private boolean isTimerOn = false;
	
	public void iterateMatchTimer() {
		
		if( !isTimerOn ) {									//sets the timer in the first call of the 
			startTime = System.currentTimeMillis();			//!!!!!!!!!!!!!! MAKE SURE THIS GETS CALLED IN AUTONOMOUS !!!!!!!!!!!!
			this.isTimerOn = true;							//do not put in init()....
		}
		
		if( System.currentTimeMillis() > 1000 * 148.5 ) { 				    /// 1000 miliseconds times the number of seconds from start of AUTONOMOUS !!!!
			//Robot.getRobot().getForklift().iterateBrakeMotor( true );								// the match lasts 150 secs
			
		}
	}

	@Override
	public void init() {
		// TODO Auto-generated method stub
	}

	@Override
	public void iterate() {
		iterateMatchTimer();		
		
	}

	@Override
	public void reset() {
		// TODO Auto-generated method stub
		
	}

}
