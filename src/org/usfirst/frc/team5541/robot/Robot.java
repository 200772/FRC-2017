package org.usfirst.frc.team5541.robot;

import java.util.ArrayList;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
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
	Joystick flight;
	
	int index_rightTalon = 1;
	int index_rightSlaveTalon = 2;
	int index_leftTalon = 3;
	int index_leftSlaveTalon = 4;
	
	int index_winch = 0;
	int index_winch2 = 1;
	
	int index_solenoid_left = 0;
	int index_solenoid_right = 1;
	int index_solenoid_back = 2;
	
	final String defaultAuto = "Vision Right"; //vision right
	final String customAuto = "Left"; //left
	final String customAuto2 = "Straight"; //straight
	final String customAuto3 = "Ultimate"; //ultimate
	final String customAuto4 = "Right"; //right
	final String visionLeft = "Vision Left"; //vision left
	final String placeholder = "Default (Nothing)";

	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	//SmartDashboard dash = new SmartDashboard();
	//String automodeselected;

	private VisionThread visionThread;
	private double centerX = 0.0;
	private final int cam_WIDTH = 320;
	private final int cam_HEIGHT = 240;
	
	private final Object imgLock = new Object();
	
	private ArrayList<MatOfPoint> imageMats = new ArrayList<>();
	
	VictorSP winch;
	VictorSP winch2;
	
	Solenoid solenoid_left;
	Solenoid solenoid_right;
	Solenoid solenoid_back;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
		UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
	    camera.setResolution(cam_WIDTH, cam_HEIGHT);
	    camera2.setResolution(cam_WIDTH, cam_HEIGHT);
	    
	    CvSource outputStream = CameraServer.getInstance().putVideo("Computer Vision", cam_WIDTH, cam_HEIGHT);
	    
	    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
	        if (pipeline.filterContoursOutput().size() > 0) {
	            Rect a = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
	            synchronized (imgLock) {
	            	centerX = a.x + (a.width / 2);
	            	//imageMats.clear();
	            	//imageMats.addAll(pipeline.filterContoursOutput());
	            }
	        }
	        synchronized (imgLock) {
	        	outputStream.putFrame(pipeline.cvErodeOutput());
            }
	    });
	    visionThread.start();
        
		chooser.addDefault("Right", customAuto4);
		chooser.addObject("Left", customAuto);
		chooser.addObject("Straight", customAuto2);
		chooser.addObject("Ultimate", customAuto3);
		chooser.addObject("Vision Right", defaultAuto);
		chooser.addObject("Vision Left", visionLeft);
		//chooser.addDefault("Default (Nothing)", placeholder);

		SmartDashboard.putData("Auto choices", chooser);
		
		//String[] choices = {defaultAuto, customAuto, customAuto2, customAuto3, customAuto4, visionLeft};
		//dash.putStringArray("Auto Selector", choices);
		
		stick = new Joystick(0);
		flight = new Joystick(1);
		
		CANTalon rightTalon = new CANTalon(index_rightTalon);
		CANTalon rightSlaveTalon = new CANTalon(index_rightSlaveTalon);
		rightTalon.setInverted(true);
		rightSlaveTalon.setInverted(true);
		rightTalon.reverseOutput(true);
		rightSlaveTalon.reverseOutput(true);
		
		CANTalon leftTalon = new CANTalon(index_leftTalon);
		CANTalon leftSlaveTalon = new CANTalon(index_leftSlaveTalon);
		
		robot = new RobotDrive(rightTalon, rightSlaveTalon, leftTalon, leftSlaveTalon);
		
		/*Jack - I changed this to a victor SP controller, 
		 * it is less error prone and doesn't clutter your can bus by using a PWM controller
		 * -Josh
		 */
		winch = new VictorSP(index_winch);
		winch2 = new VictorSP(index_winch2);
		
		/* Jack
		 * When declaring the Solendoid object, you have to tell it what channel it is on
		 * Follows Solenoid(Channel, Port)
		 * -Josh
		 */
		solenoid_left = new Solenoid(1, index_solenoid_left);
		solenoid_right = new Solenoid(1, index_solenoid_right);
		solenoid_back = new Solenoid(1, index_solenoid_back);
		
		
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
	
	double direction = 1;
	double timeToRun = 0;
	
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
		//automodeselected = dash.getString("Auto Selector");
		
		//if(autoSelected == placeholder) {
		//	autoSelected = automodeselected;
		//}
		
		switch (autoSelected) {
		
		//Straight to pin
		case customAuto2:
			
			if(timer.get() <= 1.3) {
				robot.drive(-0.5, 0);
			}
			
			break;
			
		//on the left side of the field
		//TURNS RIGHT
		case customAuto4:
			
			if(timer.get() <= 1.16) {
				robot.drive(-0.6, 0);
			} 
			if(timer.get() > 1.16 
					&& timer.get() <= 1.56) {
				robot.drive(-0.5, 0.9);
			} 
			if(timer.get() > 1.56 
					&& timer.get() <= 2) {
				robot.drive(-0.5, 0);
			}
			if(timer.get() > 2) {
				robot.drive(0, 0);
			}
			
			break;
			
		//on the right side of the field
		//TURNS LEFT
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
			
		//TURNS LEFT
		case visionLeft:
			//Jack's Autonomous
			
			//Drive forward when target is found
			if(stopped) {
				if(timer.get() <= 2) {
					robot.drive(-0.25, 0);
				}
				return;
			}
			
			//Initial Routing to peg
			if(timer.get() <= 1.3) {
				robot.drive(-0.5, 0);
			} 
			if(timer.get() > 1.3 
					&& timer.get() <= 1.7) {
				robot.drive(-0.5, -0.9);
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
				//Vision Tracking
				double centerX;
				ArrayList<MatOfPoint> points;
				double speed = -0.05;
				
				synchronized (imgLock) {
					centerX = this.centerX;
					points = (ArrayList<MatOfPoint>) this.imageMats.clone();
				}
				
				double turn = 0;
				
				//If amount of points is exactly 2 then continue
				if(points.size() == 2) {
					
					//Check if the 2 points are similar enough to be the goal
					Rect a = Imgproc.boundingRect(points.get(0));
					Rect b = Imgproc.boundingRect(points.get(1));
					
					double give = 0.1;
					//Check similar width
					if(a.width + (a.width * give) > b.width &&
							a.width - (a.width * give) < b.width) {
						//Check similar height
						if(a.height + (a.height * give) > b.height &&
								a.height - (a.height * give) < b.height) {
							//Both height and width are similar enough
							//Find center point between the 2 rects
							double ca = a.x + (a.width / 2);
							double cb = b.x + (b.width / 2);
							
							double dual = (ca + cb) / 2;
							
							turn = dual - (cam_WIDTH / 2);
							
							stopped = true;
							timer.reset();
							
						} else { turn = timer.get() % 2 == 0?50:-50; }
					} else { turn = timer.get() % 2 == 0?50:-50; }
					
					/*
					double turn_threashold = 0.4;
					double turn_con = turn * 0.005;
					
					//If the turn size is greater than threashold then limit it
					if(Math.abs(turn_con) > turn_threashold) {
						turn = turn_con<0?-turn_threashold:turn_threashold;
					}
					*/
				} else {
					//Scan back and forth at 0.25 turn
					turn = timer.get() % 2 == 0?30:-30;
				}
				
				robot.arcadeDrive(speed, turn * 0.005);

			}
			//System.out.println(turn + " : " + (turn_converted));
			break;
			
		//on the left side of the field + vision processing
		//TURNS RIGHT
		case defaultAuto:
		default:
			//Jack's Autonomous
			
			//Drive forward when target is found
			if(stopped) {
				if(timer.get() <= 2) {
					robot.drive(-0.25, 0);
				}
				return;
			}
			
			//Initial Routing to peg
			if(timer.get() <= 1.3) {
				robot.drive(-0.5, 0);
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
				//Vision Tracking
				double centerX;
				ArrayList<MatOfPoint> points;
				double speed = -0.05;
				
				synchronized (imgLock) {
					centerX = this.centerX;
					points = (ArrayList<MatOfPoint>) this.imageMats.clone();
				}
				
				double turn = timer.get() % 2 == 0?50:-50;
				
				//If amount of points is exactly 2 then continue
				if(points.size() == 2) {
					
					//Check if the 2 points are similar enough to be the goal
					Rect a = Imgproc.boundingRect(points.get(0));
					Rect b = Imgproc.boundingRect(points.get(1));
					
					double give = 0.1;
					//Check similar width
					if(a.width + (a.width * give) > b.width &&
							a.width - (a.width * give) < b.width) {
						//Check similar height
						if(a.height + (a.height * give) > b.height &&
								a.height - (a.height * give) < b.height) {
							//Both height and width are similar enough
							//Find center point between the 2 rects
							double ca = a.x + (a.width / 2);
							double cb = b.x + (b.width / 2);
							
							double dual = (ca + cb) / 2;
							
							turn = dual - (cam_WIDTH / 2);
							
							stopped = true;
							timer.reset();
							
						}
					}
				}
				
				robot.arcadeDrive(speed, turn * 0.005);

			}
			//System.out.println(turn + " : " + (turn_converted));
			break;
			
		//Intelligent autonomous
		//ARNAUDS
		case customAuto3:
			//Arnaud's Autonomous
			double centerX;
			
			synchronized (imgLock){
				centerX = this.centerX;
			}
			System.out.println("centerX :" + centerX);
			System.out.println("time to run :" + timeToRun);

			double distCenter = centerX - (cam_WIDTH/2);
			
			if(distCenter > 0){
				direction=1;
			}
			else if(distCenter < 0){
				direction=-1;
			}
			
			if(timer.get()<0.4){ // all using trigonometry rules (SOH CAH TOA)
				double referenceDistance = (cam_WIDTH/2)/Math.tan(34.25);//The reference distance is a distance to have a second fixed value to be able to calculate the angle with which the camera detects the pin.
				double angleToPin = Math.atan(distCenter/referenceDistance); //when we use the tg formula we can find the length (in pixels) of the adjacent side of the triangle (adj=320/tg(34.25 degree)=469.98) which we use to find the angle between the the view to the center and the view of the point
				double realDisToPin = Math.tan( angleToPin) / 88.12; //88.12in is the distance from the front of the robot to horizontal line of the pin. We use that measure with the angle we found earlier to have the distance between the intersection point of the robot with the horizontal line and the pin.
				double heightPinTurnPoint = Math.tan(30) * realDisToPin; //This the calculate the vertical distance between the pin and the point where the robot would have to do a 30 degree turn to get to the pin.
				double heightBeforeTurn= 88.12 - heightPinTurnPoint; //Using the distance previously calculated we subtract it from the total distance to get the distance the robot have to travel before having to turn.
				timeToRun = (0.01452 * heightBeforeTurn) + 0.4; //We convert here the distance the robot have to travel in time. Based on measurements the robot is traveling 1 inch in 0.01452 sec.
			}
			
			if(timer.get() <= timeToRun) {
				robot.drive(-0.5, 0);
			}
			if(timer.get() >timeToRun
					&& timer.get() <= timeToRun+0.4){
				robot.drive(-0.5, direction * 0.9); //angular speed to be calculated!!!
			}
			
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
		
		double speed = Math.abs(flight.getRawAxis(1));
		winch.set(speed);
		winch2.set(speed);
		
		//A = 1
		//B = 2
		//TODO Map to flight stick, add safety to prevent double activation
		/*int frontSolenoidToggle = 0;
		if(stick.getRawButton(1) && frontSolenoidToggle == 0) {
			frontSolenoidToggle = 1;
		} else if (stick.getRawButton(1) && frontSolenoidToggle == 1) {
			frontSolenoidToggle = 0;
		}*/
		//Left and right solenoid
		/*
		if(stick.getRawButton(1)) {
			solenoid_left.set(true);
			solenoid_right.set(true);
		} else if (stick.getRawButton(2)) {
			solenoid_left.set(false);
			solenoid_right.set(false);
		}
		
		//Back solenoid
		if(stick.getRawButton(2)) {
			solenoid_back.set(true);
		} else {
			solenoid_back.set(false);
		}
		*/
		/*
		if(flight.getRawAxis(1) > 0.1) {
			solenoid.set(true);
		} else {
			solenoid.set(false); 
		}
		*/
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
