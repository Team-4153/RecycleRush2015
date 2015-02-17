package org.usfirst.frc.team4153.robot;


import java.io.File;

import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.usfirst.frc.team4153.robot.subsystems.Chassis;
import org.usfirst.frc.team4153.robot.subsystems.Forklift;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	private static Robot robot;

	public static Robot getRobot () {
		return robot;
	}

	Chassis chassis;
	Forklift forklift;
	Sensors sensors;
	USBCamera usbCamera;
	Joystick manipulatorJoystick;
	VideoCapture capture; 
	int lastOpenedCamera = 0;

	static long autoTime = System.currentTimeMillis();

	static {
		System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		robot = this;

		// This line when deployed seems to get rid of the sticky fault
		// but prevents robot code from happening

		//PDPJNI.clearPDPStickyFaults(IntBuffer.wrap(new int[]{0}));

		//Joystick driveStick = new Joystick(RobotMap.DRIVER_JOYSTICK);

		sensors = new Sensors();
		sensors.init();

		chassis = new Chassis();
		chassis.init();

		forklift = new Forklift();
		forklift.init();


		//usbCamera = new USBCamera("cam1");
		//usbCamera.openCamera();
		//CameraServer.getInstance().startAutomaticCapture(usbCamera);

		manipulatorJoystick = new Joystick(RobotMap.MANIPULATOR_JOYSTICK);

		// create a new video capture object.
		capture = new VideoCapture();
		//capture.open(0, 640, 480, 1);
		// open the camera.   If you plug/unplug cameras the device number (the first parameter)
		// may change.  Zero and one seem popular.
		// If you putty in you can check:
		// ls -al /dev/video*
		// and the number SHOULD correspond to the last digit 
		// (ie /dev/video0 should be zero and /dev/video1 should be one)
		// I would assume that it's possible to put more cameras on the rio (usb hub required)
		// and so read 2,3,4,5...
		capture.open(1, 640, 480, 10);

	}

	@Override
	public void disabledPeriodic() {

		saveJpg();

		Sensors.resetIntegration();

		autoTime = System.currentTimeMillis();

		////////////////////////////////////
		//		try {
		//			//System.out.println ("Capturing Video");
		//			VideoCapture capture = new VideoCapture(1); // open the default camera
		//
		//			Mat fromCamera = new Mat();
		//			capture.retrieve(fromCamera);
		//
		//			Highgui.imwrite("/var/local/natinst/www/images/" + "Test.jpg", fromCamera);
		//
		//			//			ByteBuffer imgbuf = ByteBuffer.wrap(new byte[fromCamera.cols()*fromCamera.rows()*(int)fromCamera.elemSize()]);
		//			//			Image frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);        
		//			//			NIVision.imaqArrayToImage(frame, new RawData(imgbuf), fromCamera.cols(), fromCamera.rows());
		//			//			CameraServer.getInstance().setImage(frame);
		//		} catch ( Exception any ) {
		//			any.printStackTrace();
		//		}

		//////////////////////////////
	}



	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		int mode = (int) SmartDashboard.getNumber("AutoMode");
		long currentTime = System.currentTimeMillis()-autoTime;
		//forklift.autoPeriodic();
		switch(mode) {

		case RobotMap.DRIVE_FORWARD_MODE:
			chassis.autoDrive( 0.0 , -0.5, 0 );
			break;
		case 1:
			if (currentTime <=300) {
				forklift.open();
				Sensors.getGyro().reset();
				// do nichts
				// wait for the jaws to open

			} else if(currentTime <= 400) { 
				forklift.calibrate();
			}else if(currentTime <= 1500) { 
				if (!forklift.isCalibrating()) {
					forklift.close();
				}
			}else if (currentTime <= 2100) {
				if (!forklift.isCalibrating()) {
					forklift.close();
				}
				forklift.moveTo(2000);
				if (Sensors.getGyroAngle()>-90) {
					chassis.autoDrive(0, 0, -0.3);
				}else {
					chassis.autoDrive(0, 0, 0);
				}

			} else if (currentTime <= 3100) {

				if (Sensors.getGyroAngle()>-90) {
					chassis.autoDrive(-0.5, 0, -0.3);
				}else {
					chassis.autoDrive(-0.5, 0, 0);
				}

			} else if (currentTime>=4300) {

				if (Sensors.getGyroAngle()<0) {
					chassis.autoDrive(0, 0, 0.25);
				}else {
					chassis.autoDrive(0, 0, 0);
				}
			}
			break;
		case 2:
			if (currentTime<=300) {
				forklift.open();
				Sensors.getGyro().reset();
				// do nichts
				// wait for the jaws to open
				forklift.setPercentVBusMode(true);
			} else if(currentTime <= 1400) { 
				forklift.applyBrake(false);
				forklift.moveTo(-0.2);
			}else if(currentTime <= 1700) { 
				forklift.applyBrake(true);
				forklift.moveTo(0);
				forklift.close();
			}else if (currentTime <= 2700) {
				forklift.applyBrake(false);
				forklift.moveTo(0.9);
			} else if (currentTime <= 3400) {
				forklift.moveTo(0.9);
				
				chassis.autoDrive(0.6, 0.05, 0);
			} else if (currentTime <= 4200) {
				forklift.moveTo(0);
				forklift.applyBrake(true);

				forklift.setPercentVBusMode(false);
				forklift.moveTo(forklift.getPosition());
				chassis.autoDrive(0., -0.5, 0);
			} else if (currentTime>=5000) {
				if (Sensors.getGyroAngle()<90) {
					chassis.autoDrive(0, 0, 0.25);
				}else {
					chassis.autoDrive(0, 0, 0);
				}
			}

		}

	}



	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		chassis.iterate();	
		forklift.iterate();
		sensors.iterate();



	}

	/** Read an image from the USB camera to a JPG on the disk. */
	public void saveJpg () {
		Mat fromCamera = new Mat();
		try {
			//			if ( capture == null ) {
			//				//System.out.println ("Capturing Video");
			//				//VideoCapture capture; // open the default camera
			//				// open the default camera
			////				for ( int cameraNumber = lastOpenedCamera ; cameraNumber < 10 ; cameraNumber++ ) {
			////					try {
			////						capture = new VideoCapture(cameraNumber);
			////						if ( capture.isOpened() ) { // check if we succeeded
			////							lastOpenedCamera = cameraNumber + 1;
			////							break;
			////						}
			////					} catch ( Exception any ) {
			////						any.printStackTrace();
			////					}
			////				}
			//				//capture = new VideoCapture();
			//				//capture.open(0, 640, 480, 1);
			//			}
			if ( capture != null && capture.isOpened() ) { // check if we succeeded

				// read from the camera.
				boolean success = capture.read(fromCamera);
				//System.out.println ("Success " + success + " " + System.currentTimeMillis() % 10000);
				if (success ) {
					// works on RoboRio and be accessible from the web browser (http://roborio-4153.local/Test.jpg)
					// there seems to be a problem here in that the image gets overwritten
					// and the web server will see half an image.
					File f = new File ("/var/local/natinst/www/images/" + "Temp.jpg");
					Highgui.imwrite(f.getAbsolutePath(), fromCamera);
					f.renameTo(new File("/var/local/natinst/www/images/" + "Test.jpg"));
				}

				// Should convert from an OpenCV image to an NIVision Image and so can be sent to the dashboard.
				// This code doesn't work.  The problem is (probably) the format of the array, but the error
				// is a null pointer exception.  
				//				byte[] imageBytes = new byte[fromCamera.cols()*fromCamera.rows()*(int)fromCamera.elemSize()];
				//				fromCamera.get(fromCamera.rows(), fromCamera.cols(), imageBytes);
				//				ByteBuffer imgbuf = ByteBuffer.wrap(imageBytes);
				//				Image frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
				//				RawData rawData = new RawData(imgbuf);
				//				NIVision.imaqArrayToImage(frame, rawData, fromCamera.cols(), fromCamera.rows());
				//				CameraServer.getInstance().setImage(frame);
			}
		} catch ( Exception any ) {
			any.printStackTrace();
		}
		fromCamera.release();
		//capture.release();
		//capture = null;
	}

	/**
	 * This function is called periodically during test mode
	 */

	public void testPeriodic() {					//code to have

		boolean tempb = SmartDashboard.getBoolean("dashButton1");
		System.out.println(tempb);
		int tempv = (int) SmartDashboard.getNumber("dashPot1",0);
		System.out.println(tempv);
		SmartDashboard.putBoolean("returnBool",SmartDashboard.getBoolean("dashButton1"));

	}
	public void disabledInit() {

		//forklift.reset();
		//forklift.calibrate();
		sensors.reset();

	}


	public Chassis getChassis() {
		return chassis;
	}



	public Forklift getForklift() {
		return forklift;
	}



	public Sensors getSensors() {
		return sensors;
	}



	public USBCamera getUsbCamera() {
		return usbCamera;
	}



	public Joystick getManipulatorJoystick() {
		return manipulatorJoystick;
	}


}
