package org.usfirst.frc.team4153.robot;


import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.usfirst.frc.team4153.robot.subsystems.Chassis;
import org.usfirst.frc.team4153.robot.subsystems.Forklift;
import org.usfirst.frc.team4153.util.Sensors;

import edu.wpi.first.wpilibj.CameraServer;
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

	{
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

		chassis = new Chassis();
		chassis.init();

		forklift = new Forklift();
		forklift.init();

		//usbCamera = new USBCamera("cam1");
		//usbCamera.openCamera();
        //CameraServer.getInstance().startAutomaticCapture(usbCamera);

		manipulatorJoystick = new Joystick(RobotMap.MANIPULATOR_JOYSTICK);

	}



	@Override
	public void disabledPeriodic() {
		try {
			System.out.println ("Capturing Video");
			VideoCapture capture = new VideoCapture(1); // open the default camera

			Mat fromCamera = new Mat();
			capture.retrieve(fromCamera);

			Highgui.imwrite("/var/local/natinst/www/images/" + "Test.jpg", fromCamera);

			//			ByteBuffer imgbuf = ByteBuffer.wrap(new byte[fromCamera.cols()*fromCamera.rows()*(int)fromCamera.elemSize()]);
			//			Image frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);        
			//			NIVision.imaqArrayToImage(frame, new RawData(imgbuf), fromCamera.cols(), fromCamera.rows());
			//			CameraServer.getInstance().setImage(frame);
		} catch ( Exception any ) {
			any.printStackTrace();
		}
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
		chassis.iterate();	

		forklift.iterate();



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
	public void disabledInit() {

		//forklift.reset();
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
