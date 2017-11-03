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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//main imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.TableKeyNotDefinedException;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;

//navX code
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

//shooter stuff
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogTriggerOutput;
import edu.wpi.first.wpilibj.Counter;

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
	
	//drivetrain
	OctocanumDrive octoDrive;
	RobotDrive drive;
	
	//motors (other than drivetrain)
	Victor shooter;
	Victor agitator;
	Victor intake;
	Victor hopperDoor;
	Victor climber; //don't know if we will have this or not
	
	//vision stuff
	UsbCamera cam0;
	UsbCamera cam1;
	CameraServer cameraServer;
	
	//navX board
//	AHRS ahrs;
	
	//chassis channels for testing purposes
	final int frontLeftChannel = 3;
	final int backLeftChannel = 2;
	final int frontRightChannel = 1;
	final int backRightChannel = 4;
	
	//shooter control stuff
	
	private Counter sturns = new Counter();
	private AnalogTrigger counter_ain = new AnalogTrigger(0);
	
	int tfc;
	int shoot_mode;
	double shoot_cmd;
	
	private static final int NOT_SHOOTING = 0;
	private static final int RAMP_UP = 1;
	private static final int SHOOTING = 2;
	private static final int RAMP_DOWN = 3;
	
	double shoot_rps;
	double shoot_rpm;
	double shoot_desired_cmd;
	
	double shoot_desired_rpm;
	double shoot_min_rpm;
	double shoot_max_rpm;
	
	double cmd_delt;
	double cmd_delt_min;
	double cmd_delt_max;
	
	double cmd_gain;
	
	int count_cmd_up;
	int count_cmd_dn;
	
	double rpm_error;
	double rpm_error_p;
	
	private final double kUpdatePeriod = 0.02; // 50 Hz
	
	//logger
	private static final Logger logger = Logger.getLogger(Robot.class.getName());
	
	//Other stuff
	Timer time;
	SendableChooser chooser;
	
	public Robot() {
		octoDrive = new OctocanumDrive(3, 2, 1, 4); // Placeholder motor controller ports
//		drive = new RobotDrive(3,2,4,1);
//		drive.setExpiration(0.1);
		octoDrive.setExpiration(0.1);
		driveStick = new XboxController(0);
		controlStick = new Joystick(1);
		controlScheme = new JudyControls(0, 1);
		
		//motor controllers (besides chassis)
		/*shooter = new Victor(6); //all numbers are placeholder motor controller ports
		agitator = new Victor(5);
		intake = new Victor(8);
		hopperDoor = new Victor(7);*/
		climber = new Victor(8);
		
		//pneumatics
		frontLeftS = new Solenoid(1);
		backLeftS = new Solenoid(2);
		frontRightS = new Solenoid(3);
		backRightS = new Solenoid(4);
//		compressor = new Compressor();
//		compressor.start();
		
		// Initiate drive state as Tank:
		setDriveType(DriveType.TANK);
		
		//vision stuff
		cameraServer = CameraServer.getInstance();
		cam0 = cameraServer.startAutomaticCapture(0);
		cam1 = cameraServer.startAutomaticCapture(1);
		
		//navX board
//		try {
//			ahrs = new AHRS(SPI.Port.kMXP);
//		} catch (RuntimeException ex) {
//			DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
//		}
		
		//shooter values
		tfc = 0;
		shoot_mode = NOT_SHOOTING;
		shoot_cmd = 0.0;
		counter_ain.setLimitsVoltage(1.0, 2.0);
		sturns.setUpSource(counter_ain.createOutput(AnalogTriggerOutput.AnalogTriggerType.kState));
		sturns.setSamplesToAverage(10);
		sturns.setDistancePerPulse(1);
		
		shoot_desired_cmd = 0.85;
		
		shoot_desired_rpm = 1650.0;
		shoot_max_rpm = 0.0;
		shoot_min_rpm = 10000.0;
		
		cmd_delt = 0.0;
		cmd_delt_min = 0.0;
		cmd_delt_max = 0.0;
		
		cmd_gain = 1.0;
		
		rpm_error = 0.0;
		rpm_error_p = 0.0;
		
		
		SmartDashboard.putNumber("TFC", tfc);
		
		count_cmd_up = 0;
		count_cmd_dn = 0;
		
		//other
		time = new Timer();
		chooser = new SendableChooser();
	}

	enum DriveType { MECANUM, TANK };
	DriveType currentDrive;
	
	/**
	 * Sets the drive type to Mecanum or Tank.  Operates solenoids.
	 * @param drive
	 */
	private void setDriveType(DriveType drive) {
		boolean solenoidValue = true;
		if (drive == DriveType.MECANUM) {
			currentDrive = DriveType.MECANUM;
			solenoidValue = false;
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
		setDriveType(DriveType.MECANUM);
	}
	
	@Override
	public void autonomous() {
        
		//Timer
		time.stop(); time.reset(); time.start();
		
		SmartDashboard.putNumber("time", time.get());
		
		while (time.get() < 3)
		{
			octoDrive.Drive(-0.7);
		}
		octoDrive.Drive(0.0);
//		
//		//shooter vars
//		sturns.reset();
//		sturns.setMaxPeriod(2000.0);
//		
//		shoot_rps = sturns.getRate();
//		shoot_rpm = shoot_rps * 60.0;
//		
//		//if on red alliance {
//			while(time.get() < 7) {
////				hopperDoor.set(1.0);
////				agitator.set(1.0);
//				
//				//shooter vars/dashboard publisher
//				SmartDashboard.putNumber("shoot_rpm", shoot_rpm);			
//				SmartDashboard.putNumber("sdist", sturns.getDistance());
//				SmartDashboard.putNumber("sget", sturns.get());
//				SmartDashboard.putBoolean("sstopped", sturns.getStopped());
//				SmartDashboard.putBoolean("Tstate", counter_ain.getTriggerState());
//				
//				shoot_mode = RAMP_UP;
//				
//				setDriveType(DriveType.MECANUM);
//				
//				if (shoot_mode == RAMP_UP) {
//					if(shoot_cmd < shoot_desired_cmd)
//			//		if (shoot_rpm < shoot_desired_rpm)
//					{
//						shoot_cmd += 0.01;
//					}
//					else
//					{
//						shoot_mode = SHOOTING;
//					//	shoot_desired_cmd = shoot_cmd;
//					}
//					shooter.set(-shoot_cmd);
//				}
//				else if(shoot_mode == SHOOTING) {
//					shooter.set(-shoot_cmd);
//					hopperDoor.set(0.8);
//					agitator.set(1.0);
//					if (time.get() < 6) {
//						//keep shooting
//					}
//					else
//					{
//						shoot_mode = RAMP_DOWN;
//					}
//				}
//				else if(shoot_mode == RAMP_DOWN) {
//					if(shoot_cmd > 0.0)
//					{
//						shoot_cmd -= 0.002;
//					}
//					else
//					{
//						shoot_mode = NOT_SHOOTING;
//					}
//					if(controlScheme.getShooter())
//					{
//						shoot_mode = RAMP_UP;
//					}
//					shooter.set(-shoot_cmd);
//				}
//				
//				hopperDoor.set(-0.1);
//				agitator.set(0.0);
//				
//				while(time.get() >= 7) {
//					octoDrive.MecanumDrive(-90, 0.7, 0);
//				}
//				
//				
//				
//				
//			}
//			
//			//if on blue alliance
//			while(time.get() < 8) {
//				octoDrive.MecanumDrive(90, 0.7, 0);
//			}
	}
	
	/*
	 * Auto plan:
	 * -Shoot 10 balls
	 * -Drive to the side
	 * 
	 * Refined controls:
	 * -Run shooter, agitator and open hopper door
	 * -Turn off all of above; close hopper door
	 * -Strafe left, turn 90 degrees, drive forward
	 */

	/**
	 * Runs the motors with arcade steering.
	 */
	@Override
	public void operatorControl() {
		
		time.stop(); time.reset();
		
		octoDrive.setSafetyEnabled(true);
		
		sturns.reset();
		sturns.setMaxPeriod(2000.0);
		
		while (isOperatorControl() && isEnabled()) {
			
			//timer stuff
			time.stop(); time.reset(); time.start();
			
			//shooter prep
			shoot_rps = sturns.getRate();
			shoot_rpm = shoot_rps * 60.0;
			
			SmartDashboard.putNumber("shoot_rpm", shoot_rpm);			
			SmartDashboard.putNumber("sdist", sturns.getDistance());
			SmartDashboard.putNumber("sget", sturns.get());
			SmartDashboard.putBoolean("sstopped", sturns.getStopped());
			SmartDashboard.putBoolean("Tstate", counter_ain.getTriggerState());
			
			tfc++;
			
			SmartDashboard.putNumber("TFC", tfc);
			
			//yaw data
//			SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw()); //send yaw data to dashboard
			
			//drivetrain control based on solenoid values
			
//			octoDrive.Drive(0.5);
//			logger.info("drive!");
			
//			drive.setLeftRightMotorOutputs(0.5, 0.5);
			
			if(currentDrive == DriveType.TANK) {
				octoDrive.TankDrive(driveStick);
			}
			else {
				// TODO: Had this as 'ContolStick' by mistake?
				octoDrive.MecanumDrive(driveStick);
				
			}
			
			//Octocanum Switcher statement based on solenoid values
			if(controlScheme.switchDrive()) {
				DriveType driveSelect = DriveType.TANK;
				if(currentDrive == DriveType.TANK) {
					driveSelect = DriveType.MECANUM;
				}
				setDriveType(driveSelect);
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				logger.info("Drivetrain enabled");
			}
			
			//shooter control loop | want to eventually integrate shooter, agitator and hopper door
//			if(controlScheme.getShooter()) {
//				shooter.set(0.8); //needs adjustment
//			}
//			else
//			{
//				shooter.set(0.0);
//			}
			
			//increase shooter speed
			/*
			if(controlScheme.getReset())
			{
				sturns.reset();
			}
			
			if(controlScheme.getRampUp())
			{
				if(count_cmd_up >= 20)
				{
					if(count_cmd_up == 20)
					{
						shoot_desired_cmd += 0.05;
						count_cmd_up = 21;
					}
				}
				else
				{
					count_cmd_up++;
				}
			}
			else
			{
				count_cmd_up = 0;
			}
			
			//decrease shooter speed
			if (controlScheme.getRampDown())
			{
				if (count_cmd_dn >= 20)
				{
					if (count_cmd_dn == 20)
					{
						shoot_desired_cmd -= 0.05;
					 	count_cmd_dn = 21;
					}
				}
				else
				{
					count_cmd_dn++;		
				}
			}
			else
			{
				count_cmd_dn = 0;
			}
			
			if (shoot_desired_cmd > 1.0)
			{
				shoot_desired_cmd = 1.0;
			}
			else if (shoot_desired_cmd < 0.0)
			{
				shoot_desired_cmd = 0.0;
			}
			
			SmartDashboard.putNumber("sdcmd", shoot_desired_cmd);
			
			if(shoot_mode == NOT_SHOOTING)
			{
				//user command to start shooter motor
				if(controlScheme.getShooter())
				{
					shoot_mode = RAMP_UP;
				}
			}
			else if(shoot_mode == RAMP_UP)
			{
				//ramping up
				if(shoot_cmd < shoot_desired_cmd)
		//		if (shoot_rpm < shoot_desired_rpm)
				{
					shoot_cmd += 0.01;
				}
				else
				{
					shoot_mode = SHOOTING;
				//	shoot_desired_cmd = shoot_cmd;
				}
				shooter.set(-shoot_cmd);
				logger.info("Ramp up mode");
			}
			else if(shoot_mode == SHOOTING)
			{
				 if(controlScheme.getShooter() == false)
				 {
					 shoot_mode = RAMP_DOWN;
				     SmartDashboard.putNumber("shoot_max_rpm", shoot_max_rpm);
				     SmartDashboard.putNumber("shoot_min_rpm", shoot_min_rpm);
				 }
				 if (shoot_rpm < shoot_min_rpm)
				 {
					 shoot_min_rpm = shoot_rpm;
				 }
				 if (shoot_rpm > shoot_max_rpm)
				 {
					 shoot_max_rpm = shoot_rpm;
				 }
			     SmartDashboard.putNumber("shoot_max_rpm", shoot_max_rpm);
			     SmartDashboard.putNumber("shoot_min_rpm", shoot_min_rpm);
			     double rpm_error_raw = shoot_desired_rpm - shoot_rpm;
			     rpm_error = 0.5 * rpm_error_p + 0.5 * rpm_error_raw;
			     rpm_error_p = rpm_error;			     
			     SmartDashboard.putNumber("rpm_error", rpm_error);
			     cmd_delt = 0.01 * rpm_error;
			     if (cmd_delt > 1.0)
			    	 cmd_delt = 1.0;
			     else if (cmd_delt < 0.0)
			    	 cmd_delt = 0.0;
			     double cmd = shoot_desired_cmd + cmd_delt;
			     SmartDashboard.putNumber("cmd", cmd);
			//     shooter.set(-cmd);
//				 shooter.set(-(shoot_desired_cmd + cmd_delt));
			     hopperDoor.set(0.1);
			     agitator.set(-0.8);
			     shooter.set(-(shoot_desired_cmd));
			     logger.info("Shooting");
			}
			else if(shoot_mode == RAMP_DOWN)
			{
				agitator.set(0.0);
//				hopperDoor.set(-0.1);
			 	shoot_max_rpm = 0.0;
			 	shoot_min_rpm = 10000.0;
				if(shoot_cmd > 0.0)
				{
					shoot_cmd -= 0.002;
				}
				else
				{
					shoot_mode = NOT_SHOOTING;
				}
				if(controlScheme.getShooter())
				{
					shoot_mode = RAMP_UP;
				}
				shooter.set(-shoot_cmd);
			}
			
			SmartDashboard.putNumber("mode", shoot_mode);
			SmartDashboard.putNumber("shoot_cmd", shoot_cmd);
			
			
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
			if(controlScheme.getAgitatorRight()) {
				agitator.set(1.0);
			}
			else
			{
				agitator.set(0.0);
			}
			
			//agitator control | clockwise
			if(controlScheme.getAgitatorLeft()) {
				agitator.set(-1.0);
			}
			else
			{
				agitator.set(0.0);
			}
			
			//door control
			if(controlScheme.getDoorOpen()) {
				hopperDoor.set(0.1);
			}
			if(controlScheme.getDoorClose()) {
				hopperDoor.set(-0.1);
			}
			*/
			
			//climber control
			if(controlScheme.getClimb()) {
				climber.set(-1.0);
				//ignore this line
				logger.info("Climber on");
			}
			else	
			{
				climber.set(0.0);
				logger.info("Climber off");
			}
			
//			if (controlScheme.getClimbDown()) {
//				climber.set(1.0);
//			}
//			else
//			{
//				climber.set(0.0);
//			}
			Timer.delay(kUpdatePeriod); // wait for a motor update time
		}
	}

	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
	}
}
