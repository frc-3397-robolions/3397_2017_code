package org.usfirst.frc.team3397.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
//imports
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

public class MecanumDrive {
	
	RobotDrive chassis;
	
	JudyControls controlScheme = new JudyControls(0, 1);
	
	public MecanumDrive(int frontLeft, int backLeft, int frontRight, int backRight)
	{
		chassis = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
	}
	
	public void setExpiration(double timeout)
	{
		chassis.setExpiration(timeout);
	}
	
	public void setSafetyEnabled(boolean enabled)
	{
		chassis.setSafetyEnabled(enabled);
	}
	
	public void MecanumDrive(XboxController DStick)
	{
//		double forward = DStick.getRawAxis(1);
//		double turn = DStick.getRawAxis(TurningAxis);
//		double strafe = DStick.getRawAxis(StrafingAxis);
		double forward = controlScheme.getForward();
		double turn = controlScheme.getTurn();
		double strafe = controlScheme.getStrafe();
		
		double nonTurboSpeed = 0.7;
		
		double deadzone = 0.3;
		
		forward *= -1;
		turn *= -1;
		strafe *= -1;
		
		if(Math.abs(turn) == 1)
		{
			turn *= .7; //slow down pivot speed
		}
		
		if (Math.abs(forward) < deadzone)
		{
			forward = 0.0;
		}
		
		if (Math.abs(turn) < deadzone)
		{
			turn = 0.0;
		}
		
		if (Math.abs(strafe) < deadzone)
		{
			strafe = 0.0;
		}
		
		double speedMultiplier = nonTurboSpeed + (1 - nonTurboSpeed) * DStick.getRawAxis(2);
		forward *= speedMultiplier;
		strafe *= speedMultiplier;
		
		chassis.mecanumDrive_Cartesian(strafe, forward, turn, 0);
		
	}
	
}
