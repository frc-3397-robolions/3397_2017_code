package org.usfirst.frc.team3397.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class JudyControls {
	XboxController xboxController;
	Joystick joystickController;
	
	/* Controller Map:
	 * 
		 * Drive Controller
			 * Left Trigger y axis = drive forward in tank; drive forward in mecanum
			 * Left Trigger x axis = nothing in tank; strafe in mecanum
			 * Right Trigger x axis = turn in tank; turn in mecanum
			 * Right Bumper (6) = drivetrain switcher
			 * Y button (4) = camera switcher
		 * Mechanism Controller
			 * Trigger (1) = shoot | we will have to develop a more extensive control loop for this
			 * Down button (2) = intake
			 * Up button (3) = outtake (intake in reverse direction)
			 * Left button (4) = agitator manual control (counterclockwise)
			 * Right button (5) = agitator manual control (clockwise)
			 * lower top left button (6) = manual door open (up)
			 * lower bottom left button (7) = manual door close (down)
		 * 
	 */
	
	public JudyControls(int xboxPort, int joystickPort) {
		xboxController = new XboxController(xboxPort);
		joystickController = new Joystick(joystickPort);
	}
	
	// XBox controls
	
	boolean getCamera() {
		return xboxController.getYButton();
	}
	
	boolean switchDrive() {
		return xboxController.getBumper(Hand.kRight);
	}
	
	double getForward() {
		return xboxController.getY(Hand.kLeft);
	}
	
	double getStrafe() {
		return xboxController.getX(Hand.kLeft);
	}
	
	double getTurn() {
		return xboxController.getX(Hand.kRight);
	}
	
	// Joystick controls
	public final boolean getShooter() {
		return joystickController.getTrigger();
	}
	
	boolean getIntake() {
		return joystickController.getRawButton(2);
	}
	
	boolean getOuttake() {
		return joystickController.getRawButton(3);
	}
	
	boolean getAgitatorLeft() {
		return joystickController.getRawButton(4);
	}
	
	boolean getAgitatorRight() {
		return joystickController.getRawButton(5);
	}
	
	boolean getDoorOpen() {
		return joystickController.getRawButton(6);
	}
	
	boolean getDoorClose() {
		return joystickController.getRawButton(7);
	}
	
	boolean getClimb() {
		return joystickController.getRawButton(8);
	}
	
	
	
}
