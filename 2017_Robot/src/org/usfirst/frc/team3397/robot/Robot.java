/* This is FRC Team 3397's final 2017 robot code  */
/* Any extra documentation text can go here later */

/* Controller Map:
 * 
	 * Drive Controller
		 * Left Trigger y axis = drive forward in tank; drive forward in mecanum
		 * Left Trigger x axis = nothing in tank; strafe in mecanum
		 * Right Trigger x axis = turn in tank; turn in mecanum
		 * Left Bumper (5) = Drivetrain Switcher
		 * Right Bumper (6) = Camera Switcher
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

package org.usfirst.frc.team3397.robot;
/* Imports */
//other stuff
import java.util.logging.Logger;

//vision stuff
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import org.opencv.core.Mat;
import edu.wpi.first.wpilibj.CameraServer;

//pneumatics
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

//main imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;

//navX code
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends SampleRobot {

	// pneumatic components
	Solenoid frontLeftS;
	Solenoid backLeftS;
	Solenoid frontRightS;
	Solenoid backRightS;
	Compressor compressor;
	
	//controllers
	XboxController driveStick;
//	Joystick driveStick;
	Joystick controlStick;
	JudyControls controlScheme;
	
	//control map (COMING IF TIME IS AVAILABLE)
	
	//drivetrains
	TankDrive tankDrive;
	MecanumDrive mecanumDrive;
	
	//motors (other than drivetrain)
	Victor shooter;
	Victor agitator;
	Victor intake;
	Servo hopperDoor;
	Victor climber; //don't know if we will have this or not
	
	//vision stuff
	UsbCamera cam0;
	UsbCamera cam1;
	CameraServer cameraServer;
	
	//navX board
	AHRS ahrs;
	
	//chassis channels for testing purposes
//	final int frontLeftChannel = a;
//	final int backLeftChannel = b;
//	final int frontRightChannel = c;
//	final int backRightChannel = d;
	
	//logger
	private static final Logger logger = Logger.getLogger(Robot.class.getName());
	
	public Robot() {
		tankDrive = new TankDrive(0, 1, 2, 3); // Placeholder motor controller ports
		mecanumDrive = new MecanumDrive(0, 1, 2, 3); // Placeholder motor controller ports
		tankDrive.setExpiration(0.1);
		mecanumDrive.setExpiration(0.1);
		driveStick = new XboxController(0);
		controlStick = new Joystick(1);
		controlScheme = new JudyControls(0, 1);
		
		//motor controllers (besides chassis)
		shooter = new Victor(8); //all numbers are placeholder motor controller ports
		agitator = new Victor(9);
		intake = new Victor(10);
		hopperDoor = new Servo(11);
		climber = new Victor(12);
		
		//pneumatics
		frontLeftS = new Solenoid(0);
		backLeftS = new Solenoid(1);
		frontRightS = new Solenoid(2);
		backRightS = new Solenoid(3);
		compressor = new Compressor();
		compressor.start();
		
		// Initiate drive state as Tank:
		setDriveType(DriveType.TANK);
		
		//vision stuff
		cameraServer = CameraServer.getInstance();
		cam0 = cameraServer.startAutomaticCapture(0);
		cam1 = cameraServer.startAutomaticCapture(1);
		
		//navX board
		try {
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
		}
		
	}

	enum DriveType { MECANUM, TANK };
	DriveType currentDrive;
	
	/**
	 * Sets the drive type to Mecanum or Tank.  Operates solenoids.
	 * @param drive
	 */
	private void setDriveType(DriveType drive) {
		boolean solenoidValue = false;
		if (drive == DriveType.MECANUM) {
			currentDrive = DriveType.MECANUM;
			solenoidValue = true;
		}
		else {
			currentDrive = DriveType.TANK;
		}
		
		logger.info("Setting DriveType to: " + currentDrive);
		frontLeftS.set(solenoidValue);
		backLeftS.set(solenoidValue);
		frontRightS.set(solenoidValue);
		backRightS.set(solenoidValue);
	}
	
	@Override
	public void robotInit() {
		
		cam0.setResolution(320, 240);
		cam0.setFPS(15);
		cam1.setResolution(320, 240);
		cam1.setFPS(15);
		
		Thread t = new Thread(() -> {
					
			/* Section 1 - Vision Stuff */
			//camswitcher thread
			
			CvSink cvSink0 = cameraServer.getVideo(cam0);
			CvSink cvSink1 = cameraServer.getVideo(cam1);
			CvSource outputStream = cameraServer.putVideo("Switcher", 320, 240);
			
			Mat image = new Mat();
			
			CvSink currentCvSink = cvSink0;
			
			while(!Thread.interrupted()) {
				if(controlScheme.getCamera()) {
					if(currentCvSink == cvSink1)
					{
						cvSink1.setEnabled(false);
						cvSink0.setEnabled(true);
						logger.info("Enabling cam0");
						currentCvSink = cvSink0;
					}
					else
					{
						cvSink0.setEnabled(false);
						cvSink1.setEnabled(true);
						logger.info("Enabling cam1");
						currentCvSink = cvSink1;
					}
				}
				currentCvSink.grabFrame(image);
				outputStream.putFrame(image);
			}
		});
		t.start();
		
		//vision tracking thread - will be implemented later
		
		/* Section 2 - Pneumatic Setup */
		setDriveType(DriveType.TANK);
	}
	
	@Override
	public void autonomous() {
		//No code currently, will come soon
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	@Override
	public void operatorControl() {
		
		tankDrive.setSafetyEnabled(true);
		mecanumDrive.setSafetyEnabled(true);
		
		
		while (isOperatorControl() && isEnabled()) {
			
			//yaw data
			SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw()); //send yaw data to dashboard
			
			//drivetrain control based on solenoid values
			if(currentDrive == DriveType.TANK) {
				tankDrive.TankDrive(driveStick);
			}
			else {
				// TODO: Had this as 'ContolStick' by mistake?
				mecanumDrive.MecanumDrive(driveStick);
			}
			
			//Octocanum Switcher statement based on solenoid values
			if(controlScheme.switchDrive()) {
				DriveType driveSelect = DriveType.TANK;
				if(currentDrive == DriveType.TANK) {
					driveSelect = DriveType.MECANUM;
				}
				setDriveType(driveSelect);
			}
			
			//shooter control loop | want to eventually integrate shooter, agitator and hopper door
			if(controlScheme.getShooter()) {
				shooter.set(0.8); //needs adjustment
			}
			else
			{
				shooter.set(0.0);
			}
			
			//intake control
			if(controlScheme.getIntake()) { //controlStick.getButton(ButtonType.kNumButton);
				intake.set(0.9); //needs adjustment
			}
			else
			{
				intake.set(0.0);
			}
			
			//outtake control
			if(controlScheme.getOuttake()) {
				intake.set(-0.9);
			}
			else
			{
				intake.set(0.0);
			}
			
			//agitator control | counterclockwise
			if(controlScheme.getAgitatorLeft()) {
				agitator.set(1.0);
			}
			else
			{
				agitator.set(0.0);
			}
			
			//agitator control | clockwise
			if(controlScheme.getAgitatorRight()) {
				agitator.set(-1.0);
			}
			else
			{
				agitator.set(0.0);
			}
			
			//door control
			if(controlScheme.getDoorOpen()) {
				hopperDoor.set(1.0);
			}
			else if(controlScheme.getDoorClose()) {
				hopperDoor.set(-1.0);
			}
			
			//climber control
			if(controlScheme.getClimb()) {
				climber.set(1.0);
			}
			else
			{
				climber.set(0.0);
			}
			
			Timer.delay(0.005); // wait for a motor update time
		}
	}

	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
	}
}
