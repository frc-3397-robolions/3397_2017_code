package org.usfirst.frc.team3397.robot;

public interface TankDriveInterface {
	final double nonTurboSpeed = 0.7;
	
	//void DegreeTurn(double degree, double powerLevel);
	
	void Drive(double length);
}
