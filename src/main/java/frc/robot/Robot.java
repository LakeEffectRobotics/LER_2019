/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.ControlType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Lights.Colour;

public class Robot extends TimedRobot {

	public static final Drivetrain drivetrain = new Drivetrain();
	public static final Gyro gyro = new Gyro();
	public static final Climber climber = new Climber();
	public static final Lights lights = new Lights();
	public static OI oi;

	public static final IntakeArm intakeArm = new IntakeArm();
	public static final IntakeRoller intakeRoller = new IntakeRoller();
	public static final Elevator elevator = new Elevator();
	public static final Outtake outtake = new Outtake();
	
	public static UsbCamera jevois;

	public void robotPeriodic() {
		//Called periodically, use to interface with dashboard
		SmartDashboard.putBoolean("Left Line Detector", RobotMap.leftTapeSensor1.isOnTape());
		SmartDashboard.putBoolean("Right Line Detector", RobotMap.rightTapeSensor1.isOnTape());
		SmartDashboard.putNumber("Left Encoder", RobotMap.leftDriveSpark1.getEncoder().getPosition());
		SmartDashboard.putNumber("Right Encoder", RobotMap.rightDriveSpark1.getEncoder().getPosition());
		SmartDashboard.putNumber("Elevator Encoder", RobotMap.elevatorSpark1.getEncoder().getPosition());
		SmartDashboard.putNumber("Intake Angle", RobotMap.intakeArmTalon.getSelectedSensorPosition());
		SmartDashboard.putNumber("Current Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Current Absolute Gyro Angle", gyro.getAbsoluteAngle());
		SmartDashboard.putBoolean("Camera connected", jevois.isConnected());
		//System.out.println(RobotMap.leftTapeSensor1.isOnTape()+"\t"+RobotMap.rightTapeSensor1.isOnTape());
	}

	public void enabledInit() {
		RobotMap.gyro.reset();
		Robot.lights.setColour(Lights.LEFT, Colour.PURPLE);
		Robot.lights.setColour(Lights.RIGHT, Colour.PURPLE);
	}

	@Override
	public void robotInit() {
		oi = new OI();
		oi.init();
		RobotMap.init();
		gyro.calibrate();
		lights.setBoth(Lights.Colour.PURPLE);
		intakeArm.init();
		//Setup dashboard

		//Setup jevois feed
		jevois = CameraServer.getInstance().startAutomaticCapture(1);
		jevois.setPixelFormat(PixelFormat.kRGB565);
		jevois.setResolution(320, 240);
		jevois.setFPS(30);
	}

	@Override
	public void disabledInit() {
		////
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		robotPeriodic();
		// System.out.println(RobotMap.elevatorSpark1.getEncoder().getPosition());
	}

	@Override
	public void autonomousInit() {
		enabledInit();
	}

	//private int count=0;

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		robotPeriodic();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.

		//Prevent old target height from causing issues
		RobotMap.elevatorSpark1.getPIDController().setReference(RobotMap.elevatorSpark1.getEncoder().getPosition(), ControlType.kPosition);
		Robot.elevator.setTargetHeight(RobotMap.elevatorSpark1.getEncoder().getPosition(), 0, "Enable");

		intakeArm.init();
		outtake.setSide(Outtake.LEFT, Outtake.L_IN);
		outtake.setSide(Outtake.RIGHT, Outtake.R_IN);

		enabledInit();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		robotPeriodic();

		// if(RobotMap.leftTapeSensor1.get()){
		// 	Robot.oi.xbox.setRumble(RumbleType.kLeftRumble, 1);
		// }
		// if(RobotMap.rightTapeSensor1.get()){
		// 	Robot.oi.xbox.setRumble(RumbleType.kRightRumble, 1);
		// }
		
		// System.out.println(Robot.elevator.getTargetHeight()+"\t"+RobotMap.elevatorSpark1.getEncoder().getPosition());
	}

	//3653,3967
	double l = 0.5;
	double r = 0.5;

	public void testInit() {
	}
//0.5465,0.4078,
	@Override
	public void testPeriodic() {
		// if(Math.abs(Robot.oi.xbox.getJoyLeftY()) > 0.1)
		// 	l += Robot.oi.xbox.getJoyLeftY()/200;
		// RobotMap.leftServo.set(l);
		// if(Math.abs(Robot.oi.xbox.getJoyRightY()) > 0.1)
		// 	r += Robot.oi.xbox.getJoyRightY()/200;
		// RobotMap.rightServo.set(r);
		
		// System.out.println(l+"\t"+r);
		// RobotMap.intakeArmTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		// System.out.println(RobotMap.intakeArmTalon.getSelectedSensorPosition());

		// Robot.lights.setColour(Lights.LEFT, Lights.Colour.RED);
		// Robot.lights.setColour(Lights.RIGHT, Lights.Colour.GREEN);

		// RobotMap.leftLED_PB.set(Value.kOn);
		// RobotMap.leftLED_GB.set(Value.kForward);

		// RobotMap.rightLED_PB.set(Value.kForward);
		// RobotMap.rightLED_GR.set(Value.kOn);

		Robot.lights.setColour(Lights.LEFT, Colour.PURPLE);
		Robot.lights.setColour(Lights.RIGHT, Colour.PURPLE);

		System.out.println(RobotMap.intakeLimitSwitch.get());
	}
}