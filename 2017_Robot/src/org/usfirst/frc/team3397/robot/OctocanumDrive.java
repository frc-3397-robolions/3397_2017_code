package org.usfirst.frc.team3397.robot;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//imports
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Victor;

public class OctocanumDrive implements TankDriveInterface {
	
	RobotDrive chassis;
	
	//MC Channels
	
	JudyControls controlScheme = new JudyControls(0, 1);
	
//	Victor fLChannel;
//	Victor bLChannel;
//	Victor fRChannel;
//	Victor bRChannel;
	
	final static private Logger logger = Logger.getLogger(OctocanumDrive.class.getName());
//	final int TankDriveTurningAxis = 4; //right trigger's x axis
	
	public OctocanumDrive(int frontLeft, int backLeft, int frontRight, int backRight)
	{
		chassis = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
//		logger.info("Chassis exists");
//		
//		fLChannel = new Victor(frontLeft);
//		bLChannel = new Victor(backLeft);
//		fRChannel = new Victor (frontRight);
//		bRChannel = new Victor (backRight);
	}
	
	public void setExpiration(double timeout)
	{
		chassis.setExpiration(timeout);
	}
	
	public void setSafetyEnabled(boolean enabled)
	{
		chassis.setSafetyEnabled(enabled);
	}
	
	/**
	 * This is my fancy TankDrive method
	 * @param DStick
	 */
	public void TankDrive(XboxController DStick)
	{
		double forward = controlScheme.getForward();
		double turn = controlScheme.getTurn();
		
//		turn *= -1;
		
		if(Math.abs(turn) == 1)
		{
			forward = forward * forward; //square inputs if we're turning for more sensitivity
		}								//it's less than or equal to one, so won't make anything crazy
		
		if(Math.abs(turn) == 1)
		{
			turn *= .7; //slow down pivot speed
		}
		
		// why doesn't my deadband work?

		if (Math.abs(forward) < 0.1)
		{
			forward = 0.0;
		}
		
		if (Math.abs(turn) < 0.1)
		{
			turn = 0.0;
		}
		
		double deadzone = 0.05;
		double backDeadzone = -0.05;
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
//				logger.info("Tank turning left");
//			leftSpeed = forward * turn;
//			rightSpeed = forward;
		}
		else if (turn > 0)
		{
			double value = Math.log(turn);
			double ratio = (value - m_sensitivity)/(value + m_sensitivity);
			if (ratio == 0) ratio =.0000000001;
				leftSpeed = forward;
				rightSpeed = forward / ratio;
//				logger.info("Tank turning right");
//			leftSpeed = forward;
//			rightSpeed = forward * turn;
		}
		else
		{
			leftSpeed = forward;
			rightSpeed = forward;
		}
		
		double speedMultiplier = nonTurboSpeed + (1 - nonTurboSpeed) * DStick.getRawAxis(2);
													//The more you pull on Axis2, left trigger, the larger the multiplier will be
		leftSpeed = leftSpeed * speedMultiplier;	//Currently max multiplier without turbo is 70%
		rightSpeed = rightSpeed * speedMultiplier;
//		leftSpeed *= -1;
//		rightSpeed *= -1;
		SmartDashboard.putNumber("leftSpeed", leftSpeed);
		SmartDashboard.putNumber("rightSpeed", rightSpeed);
		
		
		chassis.setLeftRightMotorOutputs(leftSpeed, rightSpeed);
		
//		chassis.tankDrive(leftSpeed, rightSpeed);
		
//		chassis.arcadeDrive(forward, turn, true);
//		chassis.
		
	}
	@Override
	public void Drive(double speed) {
		double secondsPerFoot = 1; //XXX: Depends on the power
		
		chassis.setLeftRightMotorOutputs(speed, speed);
	}
	
	public void MecanumDrive(double direction, double speed, double turn) {
		//direction is left and right
		chassis.mecanumDrive_Polar(speed, direction, turn);
	}
	
	/* Mecanum Code */
	public void MecanumDrive(XboxController DStick)
	{
		double forward = controlScheme.getForward();
		double turn = controlScheme.getTurn();
		double strafe = controlScheme.getStrafe();
		
		double nonTurboSpeed = 0.7;
		
		double deadzone = 0.3;
		
//		forward *= -1;
//		turn *= -1;
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
		
		if (strafe > 0) {
			logger.info("Mecanum speeds (strafe, forward, turn): " + strafe + ", " + forward + ", " + turn);
		}
		chassis.mecanumDrive_Cartesian(strafe, turn, forward, 0);
	}
	
	public void newMecanumDrive(XboxController DStick) {
		//Joystick axes
		double forward = controlScheme.getForward() * -1;
		double strafe = controlScheme.getStrafe() * -1;
		double turn = controlScheme.getTurn() * -1;
		
		double nonTurboSpeed = 0.7;
		double deadzone = 0.3;
		
		double output;
		
		double speedMultiplier = nonTurboSpeed * DStick.getRawAxis(2);
		
		forward *= speedMultiplier;
		strafe *= speedMultiplier;
		
		
		
//		fLChannel.set(forward);
		
		
	}
	
}


