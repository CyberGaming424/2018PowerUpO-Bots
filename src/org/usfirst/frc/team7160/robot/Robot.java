/*-Working------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                       
 * Current code                                        */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7160.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	// Limit switched
	DigitalInput grabberSwitch = new DigitalInput(9);
	DigitalInput liftUpSwitch = new DigitalInput(8);
	DigitalInput liftDownSwitch = new DigitalInput(7);
	//
	// Auton Selector
	DigitalInput leftAuton = new DigitalInput(4);
	DigitalInput rightAuton = new DigitalInput(3);
	//
	// Creates objects for the motor controllers for driving
	WPI_TalonSRX frontLeft = new WPI_TalonSRX(2);
	WPI_TalonSRX frontRight = new WPI_TalonSRX(1);
	WPI_TalonSRX backLeft = new WPI_TalonSRX(4);
	WPI_TalonSRX backRight = new WPI_TalonSRX(3);
	//
	// This object handles the motor controllers making it easier to control them
	// all at once
	MecanumDrive mainDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
	// Initial code for lift/grabber
	// Creates the object for the motor controllers that control the lift system
	WPI_TalonSRX lift1 = new WPI_TalonSRX(5);
	WPI_TalonSRX lift2 = new WPI_TalonSRX(6);
	Spark grabber1 = new Spark(0);
	Spark grabber2 = new Spark(1);
	WPI_VictorSPX grabberAngleController = new WPI_VictorSPX(7);
	//
	// Encoder and double value for the encoder
	Encoder grabberEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
	double grabberHoldDistance;
	Encoder liftEncoder = new Encoder(6, 5, false, Encoder.EncodingType.k4X);
	double liftHoldDistance;
	PIDController grabberPID = new PIDController(0.0005, 0, 0, grabberEncoder, grabberAngleController);
	//
	// Spark lock = new Spark(4);3
	// The object that handles the gyroscope
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	//
	// Creates an object for the joystick
	Joystick joy1 = new Joystick(0);
	Joystick joy2 = new Joystick(1);
	//
	// Control groups for the lift system
	SpeedControllerGroup lift = new SpeedControllerGroup(lift1, lift2);
	SpeedControllerGroup grabber = new SpeedControllerGroup(grabber1, grabber2);
	Spark PIDStore = new Spark(7);
	PIDController liftPID = new PIDController(0.05, 0, 0, liftEncoder, PIDStore);
	//
	// Timer object
	Timer timer = new Timer();
	// Auton info things
	String gameData;
	int step = 1;
	DriverStation fms = DriverStation.getInstance();
	double time;
	double gyroValue = 0.0;
	final double switchHeight = 10.0;
	final double scaleHeight = 20.0;
	double autonHeight;
	int leftAuto = 1;
	int rightAuto = 1;
	//

	public void robotInit() {
		// mainDrive.setSafetyEnabled(false);
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		gyro.calibrate();
		timer.reset();

		new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 15);
			// computer vision code not used
			/*
			 * CvSink cvSink = CameraServer.getInstance().getVideo(); CvSource outputStream
			 * = CameraServer.getInstance().putVideo("Blur", 640, 480);
			 * 
			 * Mat source = new Mat(); Mat output = new Mat();
			 * 
			 * while (!Thread.interrupted()) { cvSink.grabFrame(source);
			 * Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
			 * outputStream.putFrame(output); }
			 */
		}).start();

		grabber1.setInverted(true);
		liftPID.setOutputRange(-0.3, .3);
		PIDStore.setDisabled();
		liftPID.enable();
		grabberPID.setOutputRange(-0.1, .1);
		grabberPID.enable();
	}

	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
		gyro.reset();
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		liftPID.enable();
	}

	public void autonomousPeriodic() {
		lift.set(-liftPID.get());
		if (gyro.getAngle() >= 0.0) {
			gyroValue = gyro.getAngle();
		} else if (gyro.getAngle() < 0.0) {
			gyroValue = 360 + gyro.getAngle();
		}
		if (gyroValue >= 360 || gyroValue <= 0.0) {
			gyro.reset();
		}
		SmartDashboard.putNumber("Gyro", Math.round(gyroValue));
		time = timer.get();
		if (leftAuton.get()) {
			switch (leftAuto) {
			case 1:
				liftPID.setSetpoint(switchHeight);
				leftAuto = 2;
				break;
			case 2:
				leftAuto();
				break;
			}
		} else if (rightAuton.get())

		{
			switch (leftAuto) {
			case 1:
				liftPID.setSetpoint(switchHeight);
				leftAuto = 2;
				break;
			case 2:
				rightAuto();
				break;
			}
		} else {
			middleAuto();
		}

	}

	// Auton functions
	private void switchAuton() {
		autonHeight = switchHeight;
		switch (step) {
		case 1:
			if (time < 5)
				mainDrive.driveCartesian(0, -.4, 0);
			else {
				mainDrive.driveCartesian(0, 0, 0);
				step = 2;
			}
			break;
		case 2:
			if (time < 8 && time >= 5) {
				grabber.set(1);
			} else {
				grabber.set(0);
			}
			break;
		}
	}

	private void scaleAuton() {
		autonHeight = scaleHeight;
		switch (step) {
		case 1:
			if (time < 7)
				mainDrive.driveCartesian(0, -.4, 0);
			else {
				mainDrive.driveCartesian(0, 0, 0);
				step = 2;
			}
			break;
		case 2:
			if (time < 10 && time >= 7)
				grabber.set(.5);
			else {
				grabber.set(0);
			}
			break;
		}
	}

	private void middleAuto() {
		switch (step) {

		}
	}

	private void rightAuto() {
		if (gameData.length() > 0) {
			if (gameData.charAt(0) == 'L') {
				switchAuton();
			} else if (gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
				scaleAuton();
			}
		}
	}

	private void leftAuto() {

	}
	// Back up auton
	/*
	 * private void Auton() { if (timer.get() <= 8) { mainDrive.driveCartesian(0,
	 * -.2, -.01); } else { mainDrive.driveCartesian(0, 0, 0); }
	 * 
	 * }
	 */

	public void teleopPeriodicInit() {
		gyro.reset();
		liftHoldDistance = 0.0;
		liftEncoder.reset();
	}

	public void teleopPeriodic() {
		SmartDashboard.putNumber("Lift PID", liftPID.get());
		System.out.println(liftEncoder.get());
		System.out.println(liftHoldDistance);
		lift1.configOpenloopRamp(.2, 0);
		lift2.configOpenloopRamp(.2, 0);
		double grabberAngleDownSpeed = .2;
		double grabberAngleUpSpeed = .4;
		// Drive code
		double x = joy1.getRawAxis(1);
		double y = joy1.getRawAxis(0);
		double rot = joy1.getRawAxis(2);
		double speed = 4;

		if (joy1.getRawButton(1))
			speed = 2;
		if (Math.abs(y) >= 0.3 || Math.abs(x) >= 0.3 || Math.abs(rot) >= 0.3) {
			mainDrive.driveCartesian(y / speed, x / speed, rot / 2);
		} else {
			mainDrive.driveCartesian(0, 0, 0);
		}
		//

		// The code used for for the lift System
		double liftUpSpeed = .5;
		double liftDownSpeed = -.3;
		if (!(liftUpSwitch.get()))
			liftUpSpeed = 0;

		if (joy2.getRawButton(1)) {
			lift.set(liftUpSpeed);
			liftHoldDistance = liftEncoder.getDistance();
			liftPID.setSetpoint(liftHoldDistance);

		} else if (joy2.getRawButton(3)) {
			lift.set(liftDownSpeed);
			liftHoldDistance = liftEncoder.getDistance();
			liftPID.setSetpoint(liftHoldDistance);
		} else {
			liftPID.enable();
			lift.set(-liftPID.get());
			// lift.set(0);
		}
//
		//

		// The code used for our grabber
		double grabberIn = -1;
		if (!(grabberSwitch.get()))
			
			grabberIn = 0;
		if (joy2.getPOV() == 0)
			grabber.set(1);
		else if (joy2.getPOV() == 180)
			grabber.set(grabberIn);
		else
			grabber.set(0);
		
		if (joy2.getRawButton(3)) {
			grabberPID.disable();
			grabberAngleController.set(-grabberAngleDownSpeed);
			grabberHoldDistance = grabberEncoder.getDistance();
			grabberPID.setSetpoint(grabberEncoder.getDistance());
		} else if (joy2.getRawButton(4)) {
			grabberPID.disable();
			grabberAngleController.set(grabberAngleUpSpeed);
			grabberHoldDistance = grabberEncoder.getDistance();
			grabberPID.setSetpoint(grabberEncoder.getDistance());
		} else {
			grabberPID.enable();
			grabberAngleController.set(-grabberPID.get());
			//grabberAngleController.set(0);
		}

		//

		// Possible code for the locking mechanism
		/*
		 * if(joy1.getRawButton(3)) lock.set(.3); else lock.set(0);
		 */
		//

	}

	public void testInit() {
	}

	@Override
	public void testPeriodic() {

	}
}
