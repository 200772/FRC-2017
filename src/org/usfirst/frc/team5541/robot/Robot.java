package org.usfirst.frc.team5541.robot;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive robot;
	
	Joystick stick;
	//Joystick flight;
	
	int index_rightTalon = 1;
	int index_rightSlaveTalon = 2;
	int index_leftTalon = 3;
	int index_leftSlaveTalon = 4;
	
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

	private VisionThread visionThread;
	private double centerX = 0.0;
	private final int cam_WIDTH = 640;
	private final int cam_HEIGHT = 480;
	
	private final Object imgLock = new Object();
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		//UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
		//UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
        //camera.setResolution(640, 480);
        
        /*
		new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(640, 480);
            
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
            
            Mat source = new Mat();
            Mat output = new Mat();
            
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
            }
        }).start();
		*/
		
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
		UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
	    camera.setResolution(cam_WIDTH, cam_HEIGHT);
	    camera2.setResolution(cam_WIDTH, cam_HEIGHT);
	    
	    CvSource outputStream = CameraServer.getInstance().putVideo("Computer Vision", cam_WIDTH, cam_HEIGHT);
	    
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	    	
	        if (!pipeline.filterContoursOutput().isEmpty()) {
	            Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	            synchronized (imgLock) {
	                centerX = r.x + (r.width / 2);
	                outputStream.putFrame(pipeline.hsvThresholdOutput());
	            }
	        }
	    });
	    visionThread.start();
        
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		stick = new Joystick(0);
		//flight = new Joystick(1);
		
		CANTalon rightTalon = new CANTalon(index_rightTalon);
		CANTalon rightSlaveTalon = new CANTalon(index_rightSlaveTalon);
		CANTalon leftTalon = new CANTalon(index_leftTalon);
		CANTalon leftSlaveTalon = new CANTalon(index_leftSlaveTalon);
		
		robot = new RobotDrive(rightTalon, rightSlaveTalon, leftTalon, leftSlaveTalon);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	int autoLoopCounter;
	final int STAGE_STRAIGHT_1 = 100;
	final int STAGE_TURN_RIGHT_1 = STAGE_STRAIGHT_1 + 100;
	final int STAGE_STRAIGHT_2 = STAGE_TURN_RIGHT_1 + 50;
	
	boolean stopped;
	
	@Override
	public void autonomousInit() {
		
		stopped = false;
		
		autoLoopCounter = 0;
		
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			//Benchmark
			autoLoopCounter++;
			
			robot.drive(-1, 0);
			
			break;
		case defaultAuto:
		default:
			
			if(stopped) { return; }
			
			double centerX;
			double speed = -0.01;
			synchronized (imgLock) {
				centerX = this.centerX;
			}
			double turn = centerX - (cam_WIDTH / 2);
			double turn_converted = turn * 0.005;
			double turn_threashold = 0.4;
			
			if(Math.abs(turn_converted) > turn_threashold) {
				if(turn_converted < 0) { turn_converted = -turn_threashold; }
				if(turn_converted > 0) { turn_converted = turn_threashold; }
			}
			if(Math.abs(turn_converted) < 0.1) {
				speed = 0;
				turn_converted = 0;
				stopped = true;
			}
			
			robot.arcadeDrive(-0.01, turn_converted);
			System.out.println(turn + " : " + (turn_converted));
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//robot.arcadeDrive(flight);
		//robot.tankDrive(stick.getRawAxis(1), stick.getRawAxis(5));
		robot.arcadeDrive(stick);
		//flight stick, 0 x axis, 1 y axis
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

