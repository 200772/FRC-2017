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
import edu.wpi.first.wpilibj.Timer;
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
	
	final String defaultAuto = "right";
	final String customAuto = "left";
	final String customAuto2 = "straight";
	final String customAuto3 = "ultimate";

	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

	private VisionThread visionThread;
	private double centerX = 0.0;
	private final int cam_WIDTH = 320;
	private final int cam_HEIGHT = 240;
	
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
        
		chooser.addDefault("Right", defaultAuto);
		chooser.addObject("Left", customAuto);
		chooser.addObject("Straight", customAuto2);
		chooser.addObject("Ultimate", customAuto3);

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
	
	Timer timer = new Timer();
	
	boolean stopped;
	
	@Override
	public void autonomousInit() {
		
		stopped = false;
		
		timer.start();
		
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
		
		//Straight to pin
		case customAuto2:
			
			if(timer.get() <= 1.3) {
				robot.drive(-0.5, 0);
			}
			
			break;
			
		//on the right side of the field
		case customAuto:
			
			if(timer.get() <= 1.16) {
				robot.drive(-0.6, 0);
			} 
			if(timer.get() > 1.16 
					&& timer.get() <= 1.56) {
				robot.drive(-0.5, -0.9);
			} 
			if(timer.get() > 1.56 
					&& timer.get() <= 2) {
				robot.drive(-0.5, 0);
			}
			if(timer.get() > 2) {
				robot.drive(0, 0);
			}
			
			break;
			
		//on the left side of the field + vision processing
		case defaultAuto:
		default:
			//50 ~ 1 second
			if(stopped) {
				if(timer.get() <= 3.2) {
					robot.drive(-0.15, 0);
				}
				return;
			}
			//wouldn't we add if stopped=false? 
			if(timer.get() <= 1.3) {
				robot.drive(-0.6, 0);
			} 
			if(timer.get() > 1.3 
					&& timer.get() <= 1.7) {
				robot.drive(-0.5, 0.9);
			} 
			if(timer.get() > 1.7 
					&& timer.get() <= 1.9) {
				robot.drive(-0.5, 0);
			}
			if(timer.get() > 1.9 
					&& timer.get() <= 2.24) {
				//Do nothing
				System.out.println("Pause before search");
			}
			if(timer.get() > 2.24) {
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
				if(Math.abs(turn_converted) < 0.08) {
					speed = 0;
					turn_converted = 0;
					stopped = true;
					timer.reset();
				}
				robot.arcadeDrive(-0.01, turn_converted);

			}
			//System.out.println(turn + " : " + (turn_converted));
			break;
			
		//Intelligent autonomous
		case customAuto3:
			double centerX;
			double direction = 1;
			double timeToRun = 0;
			
			synchronized (imgLock){
				centerX = this.centerX;
			}
			System.out.println("centerX :" + centerX);

			/*double distCenter = centerX - (cam_WIDTH/2);
			
			
			if(distCenter > 0){
				direction=1;
			}
			else if(distCenter < 0){
				direction=-1;
			}
			
			
			//if(timer.get()<0.4){ //should we do this like that? with an if timer < 0.4  to be sure the calcutions are not done twice? or is there some other method to do this?
				double AngleToPin = Math.atan(distCenter/469.98);
				double RealDisToPin = Math.tan( AngleToPin) / 72.87;
				double heightPinTurnPoint = Math.tan(30) * RealDisToPin;
				double heightBeforeTurn= 72.87 - heightPinTurnPoint;
				timeToRun = 0.1452 * heightBeforeTurn; //0.1452 => value when robot was stopped
				System.out.println("time to run :" + timeToRun);
			//} */
			
			/*
			if(timer.get() <= timeToRun) {
				robot.drive(-0.5, 0);
			}
			if(timer.get() <= timeToRun+0.4){
				robot.drive(-0.5, direction * 0.9); //angular speed to be calculated!!!
			}
*/
			//to add:   - detection of how big the marks are to evaluate distance
			//			- adding alignment code (Jack)
			
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		//robot.arcadeDrive(flight);
		robot.tankDrive(stick.getRawAxis(1), stick.getRawAxis(5));
		//robot.arcadeDrive(stick);
		//flight stick, 0 x axis, 1 y axis
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

