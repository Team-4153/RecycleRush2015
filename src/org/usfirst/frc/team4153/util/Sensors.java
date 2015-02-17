package org.usfirst.frc.team4153.util;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import org.usfirst.frc.team4153.robot.RobotMap;
import org.usfirst.frc.team4153.robot.subsystems.Subsystem;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class Sensors implements Subsystem {

	private static Gyro gyro;
	private static BuiltInAccelerometer accelerometer;
	private static Joystick driverJoystick;
	private static Encoder encoder;
	private static AnalogOutput output1;
	protected static AccelerometerDistanceDoubleIntegrationThread integrationThread;



	{						//can delete if not wanted.... doesn't check every time
		//init();
		// This method should not exist: it does not make sense to have to initialize a bunch of static objects
		// I'll leave it here as a warning but it should not be used

	}

	public static void resetIntegration () {
		if (integrationThread != null) {
			integrationThread.reset();
		}
	}

	@Override
	public void init() {
		driverJoystick = new Joystick( RobotMap.DRIVER_JOYSTICK);
		output1 = new AnalogOutput(1);
		gyro = new Gyro( RobotMap.GYRO);							//if an object is constructed
		gyro.reset();
		accelerometer = new BuiltInAccelerometer();
		System.out.println("Gyro Initalized");
		//		integrationThread = new AccelerometerDistanceDoubleIntegrationThread();
		//		integrationThread.start();
	}


	public static void setAnalogOutput(double x) {
		output1.setVoltage(x);

	}


	public static Joystick getDriverJoystick() {
		return driverJoystick;
	}

	public static Gyro getGyro() {
		return gyro;
	}

	public static BuiltInAccelerometer getAccelerometer() {
		return accelerometer;
	}



	public static double getGyroAngle() {
		if( driverJoystick.getRawButton(2) ) {
			gyro.reset();
		}
		return gyro.getAngle(); // send the angle back

	}




	public void reset() {
		getGyro().reset(); 			//if disabled 
	}

	public static void gyroReset() {
		gyro.reset();
	}



	@Override
	public void iterate() {
		//System.out.println("Displacement: " + integrationThread.displacementX +", "+ integrationThread.displacementY);

	}

	/**
	 * the purpose of this thread is to run in the background and constantly
	 * approximate the speed and position of the bot by doing a numerical
	 * integration of the acceleration and speed respectively
	 * 
	 * importantly, however, it does not factor gyro angle and therefore
	 * centripetal acceleration, meaning that it will not be accurate after
	 * rotations; it should mostly just be used in autonomous
	 * 
	 * the way that it is calculated probably means that the units are 
	 * broadly arbitrary but hopefully consistent
	 * @author 4153student
	 *
	 */
	public class AccelerometerDistanceDoubleIntegrationThread extends Thread {
		// The speed and displacement are both (probably bad) approximations based on accelerometer
		public double speedX = 0;
		public double speedY = 0;
		public double displacementX = 0;
		public double displacementY = 0;

		public double lastAccX = 0;
		public double lastAccY = 0;

		public double accXError = 0;
		public double accYError = 0;

		public boolean terminate = false;

		long lastTime;
		public void run () {
			lastTime = System.nanoTime();
			while (!terminate) {
				long currentTime = System.nanoTime();
				update(currentTime);
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				lastTime = currentTime;
			}
		}



		public void reset() {
			speedX = 0;
			speedY = 0;
			displacementX = 0;
			displacementY = 0;

			lastAccX = 0;
			lastAccY = 0;

			// Makes the errors an approximate running average
			accXError = (accXError+getAccelerometer().getZ())/10;
			accYError = (accYError+getAccelerometer().getX())/10;

			lastTime = System.nanoTime();
		}

		protected void update(long currentTime) {
			// Use the z because the box is vertical
			double currentAccX = getAccelerometer().getZ()+accXError;

			// the y acceleration seems to sit around 1 when stationary
			// using x because it is sideways
			double currentAccY = getAccelerometer().getX()+accYError;

			//System.out.println("Acc " + currentAccX+ ", " +currentAccY);

			// Time difference since last calculation in 10^-4 second increments
			//long deltaT = (currentTime - lastTime)/100000;

			// Testing if assuming time increments of 1 will drift less
			long deltaT = 1;

			// Adds the average acceleration to the speeds and divides
			double newSpeedX = speedX + (((currentAccX + lastAccX)/2)*deltaT);
			double newSpeedY = speedY + (((currentAccY + lastAccY)/2)*deltaT);

			//System.out.println("speed " + newSpeedX+ ", " +newSpeedY);

			// Adds the average speeds to the displacement
			displacementX += ((newSpeedX+speedX)/2)*deltaT;
			displacementY += ((newSpeedY+speedY)/2)*deltaT;

			lastAccX = currentAccX;
			lastAccY = currentAccY;

			speedX = newSpeedX;
			speedY = newSpeedY;

			//reset();
		}
	}



}
