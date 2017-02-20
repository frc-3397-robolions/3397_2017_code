package org.usfirst.frc.team3397.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
//imports
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.XboxController;

public class TankDrive implements TankDriveInterface {
	
	RobotDrive chassis;
	
	JudyControls controlScheme = new JudyControls(0, 1);
	
//	final int TankDriveTurningAxis = 4; //right trigger's x axis
	
	public TankDrive(int frontLeft, int backLeft, int frontRight, int backRight)
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
	
	public void TankDrive(XboxController DStick)
	{
		double forward = controlScheme.getForward();
		double turn = controlScheme.getTurn();
		
		if(Math.abs(turn) == 1)
		{
			forward *= .7; //slow down pivot speed
		}
		
		// why doesn't my deadband work?

		if (Math.abs(forward) < 0.05)
		{
			forward = 0.0;
		}
		
		if (Math.abs(turn) < 0.05)
		{
			turn = 0.0;
		}
		
		double leftSpeed;
		double rightSpeed;
		
		float m_sensitivity = .5f;
	
		if (turn < 0)
		{
			double value = Math.log(-turn);
			double ratio = (value - m_sensitivity)/(value + m_sensitivity); //This is pre-existing library code
			if (ratio == 0) ratio =.0000000001;						 // I'm not familiar with its use, so just roll with it
				leftSpeed = forward / ratio;							 //  It's here in case we need to modify it
				rightSpeed = forward;
		}
		else if (turn > 0)
		{
			double value = Math.log(turn);
			double ratio = (value - m_sensitivity)/(value + m_sensitivity);
			if (ratio == 0) ratio =.0000000001;
				leftSpeed = forward;
				rightSpeed = forward / ratio;
		}
		else
		{
			leftSpeed = forward;
			rightSpeed = forward;
		}
		
		double speedMultiplier = nonTurboSpeed + (1 - nonTurboSpeed) * DStick.getRawAxis(2);
													//The more you pull on Axis2, left trigger, the larger the multiplier will be
		leftSpeed = leftSpeed * speedMultiplier * -1;	//Currently max multiplier without turbo is 70%
		rightSpeed = rightSpeed * speedMultiplier * -1;
		
		
		chassis.setLeftRightMotorOutputs(leftSpeed, rightSpeed);
		
	}
	@Override
	public void Drive(double speed) {
		double secondsPerFoot = 1; //XXX: Depends on the power
		
		chassis.setLeftRightMotorOutputs(0, 0);
	}
}


// NOTE: THIS CLASS IS NOT NEEDED FOR THE PRACTICE BOT; HOWEVER IT IS NEEDED FOR THE FINAL. //
