package org.usfirst.frc.team5541.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	
	int index_rightTalon = 1;
	int index_rightSlaveTalon = 2;
	int index_leftTalon = 3;
	int index_leftSlaveTalon = 4;
	
	int autoLoopCounter;
	
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		stick = new Joystick(0);
		
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
	@Override
	public void autonomousInit() {
		
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
			if(autoLoopCounter < 150) {
				robot.drive(1, 0);
				autoLoopCounter++;
			} else {
				robot.drive(0, 0);
			}
			break;
		case defaultAuto:
		default:
			// By default no Autonomous
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		robot.tankDrive(stick.getRawAxis(1), stick.getRawAxis(5));
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

