package org.usfirst.frc.team5541.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	
	Joystick driver;
	int autoLoopCounter;
	int rightTalonIndex = 1;
	int rightSlaveTalonIndex = 2;
	int leftTalonIndex = 3;
	int leftSlaveTalonIndex = 4;
	
	@Override
	public void robotInit() {	
		driver = new Joystick(0);
    	
    	CANTalon rightTalon = new CANTalon(rightTalonIndex);
    	CANTalon rightSlaveTalon = new CANTalon(rightSlaveTalonIndex);
    	CANTalon leftTalon = new CANTalon(leftTalonIndex);
    	CANTalon leftSlaveTalon = new CANTalon(leftSlaveTalonIndex);
    	
    	
    	myRobot = new RobotDrive(rightTalon, rightSlaveTalon, leftTalon, leftSlaveTalon);
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
	@Override
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
	@Override
    public void autonomousPeriodic() {
    	if(autoLoopCounter < 150) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(1, 1); 	// drive forwards half speed
			autoLoopCounter++;
		} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
		}
    }
    
    /**
     * This function is called once each time the robot enters
     * tele-operated mode
     */
	@Override
    public void teleopInit(){}

    /**
     * This function is called periodically during operator control
     */
	@Override
    public void teleopPeriodic() {
        myRobot.tankDrive(-(driver.getRawAxis(1)), (driver.getRawAxis(5)));
    }
    
    /**
     * This function is called periodically during test mode
     */
	@Override
    public void testPeriodic() {}
    
}