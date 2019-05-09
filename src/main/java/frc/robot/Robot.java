/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ControlType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SerialPort.FlowControl;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Lights.Colour;
import frc.robot.commands.autonomous.AutoCommandGroup;

public class Robot extends TimedRobot {
	public static final double AUTO_TIMEOUT = 5;

	public static final Drivetrain drivetrain = new Drivetrain();
	public static final Gyro gyro = new Gyro();
	public static final Climber climber = new Climber();
	public static final Lights lights = new Lights();
	public static OI oi;

	public static final Intake intake = new Intake();
	public static final Elevator elevator = new Elevator();
	public static final Outtake outtake = new Outtake();

	public static UsbCamera webcam;
	// public static UsbCamera jevois;

	public static AutoCommandGroup autonomous_command_group;

	public static enum AutonomousTarget {
		CARGO_FRONT, CARGO_1, CARGO_2, CARGO_3, ROCKET_NEAR, ROCKET_FAR, NOTHING
	}

	public static enum AutonomousStartPosition {
		LEFT, MID_LEFT, MID_RIGHT, RIGHT
	}

	public static enum AutonomousGamepiece {
		CARGO, HATCH, NOTHING
	}

	public static SendableChooser<AutonomousStartPosition> autonomous_position_chooser = new SendableChooser<AutonomousStartPosition>();
	public static SendableChooser<AutonomousTarget> autonomous_target_chooser = new SendableChooser<AutonomousTarget>();
	public static SendableChooser<AutonomousGamepiece> autonomous_gamepiece_chooser = new SendableChooser<AutonomousGamepiece>();

	double maxL = 0;
	double maxR = 0;
	public static int visionOffset = 0;
	private String t="";
	private String[] in;
	//Used to track jevois serial status
	//1 is added if available = 0
	//1 is removed otherwise
	//If it = 0, serial port is made null
	private int last5 = 0;
	private int periodicCount=0;
	private int jevois_available=0;

	@Override
	public void robotPeriodic() {
		periodicCount++;
		// Called periodically, use to interface with dashboard
		//SmartDashboard.putNumber("Left Encoder", RobotMap.leftDriveSpark1.getEncoder().getPosition());
		//SmartDashboard.putNumber("Right Encoder", RobotMap.rightDriveSpark1.getEncoder().getPosition());
		SmartDashboard.putNumber("Elevator Encoder", RobotMap.elevatorSpark1.getEncoder().getPosition());
		SmartDashboard.putNumber("Intake Angle", RobotMap.intakeArmTalon.getSelectedSensorPosition());
		SmartDashboard.putNumber("Current Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Current Absolute Gyro Angle", gyro.getAbsoluteAngle());
		// SmartDashboard.putBoolean("Camera connected", jevois.isConnected());
		// //System.out.println(RobotMap.jevoisSerial.getBytesReceived());
		SmartDashboard.putBoolean("Serial good", RobotMap.jevoisSerial != null);
		SmartDashboard.putNumber("Vision Offset", visionOffset);
		//SmartDashboard.putNumber("Reverse", oi.shawnDrive.get() ? 1 : 0);
		// System.out.println(oi.shawnDrive.get());

		// //System.out.println(RobotMap.elevatorSpark1.getEncoder().getVelocity());
		// If there are more than 10 bytes in the buffer, clear it
		/*
		 * if(RobotMap.jevoisSerial!=null){ if(RobotMap.jevoisSerial.getBytesReceived()
		 * > 100){ RobotMap.jevoisSerial.read(RobotMap.jevoisSerial.getBytesReceived());
		 * } }
		 */

		//if (Math.abs(maxL) < Math.abs(RobotMap.leftDriveSpark1.getEncoder().getVelocity()))
		//	maxL = RobotMap.leftDriveSpark1.getEncoder().getVelocity();

		//if (Math.abs(maxR) < Math.abs(RobotMap.rightDriveSpark1.getEncoder().getVelocity()))
		//	maxR = RobotMap.rightDriveSpark1.getEncoder().getVelocity();

		// System.out.println(maxL+"\t"+maxR);
		//// System.out.println(RobotMap.leftTapeSensor1.isOnTape()+"\t"+RobotMap.rightTapeSensor1.isOnTape());
		// //System.out.println(!RobotMap.intakeLimitSwitch.get()+ "\t"
		// +RobotMap.intakeArmTalon.getSelectedSensorPosition()+"\t"+Robot.intake.getTargetPosition()+"\t"+Robot.oi.xbox.getJoyRightY());

		/*if (RobotMap.jevoisSerial == null ) {
			// if (periodicCount%50==051) {
			// 	System.out.println("initializing JeVois Serial");
			// 	try {
			// 		RobotMap.jevoisSerial = new SerialPort(115200, Port.kUSB);
			// 	} catch (Exception e) {
			// 		System.out.println("JeVois error");
			// 		RobotMap.jevoisSerial = null;
			// 	}
			// }
		} else {

			jevois_available = RobotMap.jevoisSerial.getBytesReceived();
			// Update collector
			/*
			 * int sum = available; for (int i = 0; i < collector.length - 1; i++) {
			 * collector[i] = collector[i] + 1; sum += collector[i]; }
			 * collector[collector.length - 1] = available;
			 * 
			 * // If there have been <2 bits in the last 5 messages, there's an issue if
			 * (sum < 2) { offset = 0; }
			 * /

			// Parse new offset
			//System.out.println(available);
			if(jevois_available == 0) last5 ++;
			else last5 =0;
			
			//System.out.println(last5);
			if(last5 >= 150){
				System.out.println("Resetting Jevois serial");
				RobotMap.jevoisSerial.reset();
				RobotMap.jevoisSerial.close();
				//RobotMap.jevoisSerial.setFlowControl(FlowControl.kRtsCts);
				RobotMap.jevoisSerial=null;
				last5 = 0;
			}

			if (jevois_available > 0) {
				try {
					in = RobotMap.jevoisSerial.readString().split("\n");
					if (in.length>2) {
						t = in[in.length - 2];				
						if (t.length() > 1) {
							// Remove the trailing whitespace
							t = t.substring(0, t.length() - 1);
							visionOffset = Integer.parseInt(t);
						}
					}
				} catch (Exception e) {
					//vision_offset = 0;
					System.out.println("Parse Err");
				}
			}

	}*/
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
		/******************************************************************************
		 * These 2 lines have no purpose other than to invoke actions that can cause large delays
		 * the first time they are called.
		 */
		System.out.println("Ignore this: " + maxL);
		NetworkTableInstance.getDefault();
		/**************************************************************************/
		gyro.calibrate();
		lights.setBoth(Lights.Colour.PURPLE);
		intake.init();
		// Setup dashboard
		autonomous_position_chooser.setDefaultOption("Left", AutonomousStartPosition.LEFT);
		autonomous_position_chooser.addOption("Mid Left", AutonomousStartPosition.MID_LEFT);
		autonomous_position_chooser.addOption("Mid Right", AutonomousStartPosition.MID_RIGHT);
		autonomous_position_chooser.addOption("Right", AutonomousStartPosition.RIGHT);
		SmartDashboard.putData("Start Position", autonomous_position_chooser);
		autonomous_target_chooser.setDefaultOption("Front", AutonomousTarget.CARGO_FRONT);
		autonomous_target_chooser.addOption("Cargo1", AutonomousTarget.CARGO_1);
		autonomous_target_chooser.addOption("Cargo2", AutonomousTarget.CARGO_2);
		autonomous_target_chooser.addOption("Cargo3", AutonomousTarget.CARGO_3);
		autonomous_target_chooser.addOption("Rocket near", AutonomousTarget.ROCKET_NEAR);
		autonomous_target_chooser.addOption("Rocket far", AutonomousTarget.ROCKET_FAR);
		autonomous_target_chooser.addOption("Nothing", AutonomousTarget.NOTHING);
		SmartDashboard.putData("Target", autonomous_target_chooser);
		autonomous_gamepiece_chooser.setDefaultOption("Nothing", AutonomousGamepiece.NOTHING);
		autonomous_gamepiece_chooser.addOption("Hatch", AutonomousGamepiece.HATCH);
		autonomous_gamepiece_chooser.addOption("Cargo", AutonomousGamepiece.CARGO);
		SmartDashboard.putData("Gamepiece", autonomous_position_chooser);

		// Setup webcam feed
		webcam = CameraServer.getInstance().startAutomaticCapture(0);
		webcam.setPixelFormat(PixelFormat.kYUYV);
		webcam.setResolution(320, 240);
		webcam.setFPS(25);


		// Setup jevois feed
		// jevois = CameraServer.getInstance().startAutomaticCapture(0);
		// jevois.setPixelFormat(PixelFormat.kYUYV);
		// jevois.setResolution(320, 240);
		// jevois.setFPS(25);
		
		/*try {
			RobotMap.jevoisSerial = new SerialPort(115200, Port.kUSB);
		} catch (Exception e) {
			System.out.println("JeVois error");
			RobotMap.jevoisSerial = null;
		}*/

	}

	@Override
	public void disabledInit() {
		////
	}

	@Override
	public void disabledPeriodic() {
		//Scheduler.getInstance().run();
		//robotPeriodic();

		// //System.out.println(RobotMap.elevatorSpark1.getEncoder().getPosition());
	}

	@Override
	public void autonomousInit() {
		enabledInit();
		autonomous_command_group = new AutoCommandGroup();
		autonomous_command_group.start();
	}

	// private int count=0;

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		//robotPeriodic();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.

		// Prevent old target height from causing issues
		RobotMap.elevatorSpark1.getPIDController().setReference(RobotMap.elevatorSpark1.getEncoder().getPosition(),
				ControlType.kPosition);
		Robot.elevator.setTargetHeight(RobotMap.elevatorSpark1.getEncoder().getPosition(), 0, "Enable");

		intake.init();
		outtake.setSide(Outtake.SIDE_LEFT, Outtake.L_IN);
		outtake.setSide(Outtake.SIDE_RIGHT, Outtake.R_IN);
		enabledInit();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		System.out.println(RobotMap.elevatorSpark1.getEncoder().getPosition());

		//robotPeriodic();

		// if(RobotMap.leftTapeSensor1.get()){
		// Robot.oi.xbox.setRumble(RumbleType.kLeftRumble, 1);
		// }
		// if(RobotMap.rightTapeSensor1.get()){
		// Robot.oi.xbox.setRumble(RumbleType.kRightRumble, 1);
		// }

		// //System.out.println(Robot.elevator.getTargetHeight()+"\t"+RobotMap.elevatorSpark1.getEncoder().getPosition());
	}

	// 3653,3967
	double l = 0.5;
	double r = 0.5;

	AnalogInput a0 = new AnalogInput(0);

	public void testInit() {
	}

	int count = 0;
	// 0.5465,0.4078,

	TalonSRX test = new TalonSRX(10);

	double lPos = 0;
	double rPos = 0;

	@Override
	public void testPeriodic() {
		// if(Math.abs(Robot.oi.xbox.getJoyLeftY()) > 0.1)
		// l += Robot.oi.xbox.getJoyLeftY()/200;
		// RobotMap.leftServo.set(l);
		// if(Math.abs(Robot.oi.xbox.getJoyRightY()) > 0.1)
		// r += Robot.oi.xbox.getJoyRightY()/200;
		// RobotMap.rightServo.set(r);

		// //System.out.println(l+"\t"+r);
		// RobotMap.intakeArmTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		// //System.out.println(RobotMap.intakeArmTalon.getSelectedSensorPosition());

		// Robot.lights.setColour(Lights.LEFT, Lights.Colour.RED);
		// Robot.lights.setColour(Lights.RIGHT, Lights.Colour.GREEN);

		// RobotMap.leftLED_PB.set(Value.kOn);
		// RobotMap.leftLED_GB.set(Value.kForward);

		// RobotMap.rightLED_PB.set(Value.kForward);
		// RobotMap.rightLED_GR.set(Value.kOn);

		// Robot.lights.setColour(Lights.LEFT, Colour.PURPLE);
		// Robot.lights.setColour(Lights.RIGHT, Colour.PURPLE);

		// //System.out.println(RobotMap.intakeLimitSwitch.get());

		// //System.out.println(RobotMap.jevoisSerial.getBytesReceived());

		// RobotMap.leftDriveSpark1.getEncoder().setPosition(0);
		// //System.out.println(RobotMap.leftDriveSpark1.getEncoder().getPosition()+"\t"+RobotMap.leftDriveSpark2.getEncoder().getPosition());

		// // RobotMap.climberVictor.follow(RobotMap.climberTalon);

		// RobotMap.climberTalon.set(ControlMode.PercentOutput, oi.lJoy.getY());

		// //System.out.println(count);
		lPos += Robot.oi.xbox.getJoyLeftY() / 2;
		rPos += Robot.oi.xbox.getJoyRightY() / 2;

		// if(speed > 0){
		// count += RobotMap.leftOuttakeCounter.get();
		// }
		// if(speed < 0){
		// count -= RobotMap.leftOuttakeCounter.get();
		// }
		// RobotMap.leftOuttakeCounter.reset();

		// //System.out.println(!RobotMap.leftOuttakeLimit.get()+"\t"+!RobotMap.rightOuttakeLimit.get());
		// if (RobotMap.CLIMBER_ENABLED) {
		// RobotMap.climberTalon.set(ControlMode.PercentOutput, lSpeed);
		// }

		// System.out.println("L"+RobotMap.leftOuttakeTalon.getSelectedSensorPosition()+"\tR"+RobotMap.rightOuttakeTalon.getSelectedSensorPosition());

		// if(Math.abs(lSpeed) < 0.1) lSpeed = 0;
		// if(Math.abs(rSpeed) < 0.1) rSpeed = 0;

		// test.set(ControlMode.PercentOutput, lSpeed);

		// RobotMap.leftOuttakeTalon.set(ControlMode.PercentOutput, lSpeed/2);
		// RobotMap.rightOuttakeTalon.set(ControlMode.PercentOutput, rSpeed/2);
		// RobotMap.intakeArmTalon.set(ControlMode.PercentOutput, rSpeed);

		// RobotMap.rightOuttakeTalon.set(ControlMode.Position, rPos);
		// RobotMap.leftOuttakeTalon.set(ControlMode.Position, lPos);

		RobotMap.leftOuttakeTalon.set(ControlMode.PercentOutput, Robot.oi.xbox.getJoyLeftY() / 3);
		RobotMap.rightOuttakeTalon.set(ControlMode.PercentOutput, Robot.oi.xbox.getJoyRightY() / 3);
		// if(Robot.oi.xbox.getBumperL()){
		// RobotMap.leftOuttakeCounter.setReverseDirection(!RobotMap.leftOuttakeCounter.getDirection());
		// }

		if(Robot.oi.xbox.getButtonA()){
			Robot.lights.setBoth(Lights.Colour.GREEN);
		}
		if(Robot.oi.xbox.getButtonX()){
			Robot.lights.setBoth(Lights.Colour.BLUE);
		}
		if(Robot.oi.xbox.getButtonB()){
			Robot.lights.setBoth(Lights.Colour.RED);
		}
		if(Robot.oi.xbox.getButtonY()){
			Robot.lights.setBoth(Lights.Colour.OFF);
		}

		System.out.println(RobotMap.leftOuttakeTalon.getSelectedSensorPosition() + "\t"
				+ RobotMap.rightOuttakeTalon.getSelectedSensorPosition());
		// System.out.println(RobotMap.outerLeftSensor.isOnTape() + "\t" + RobotMap.innerLeftSensor.isOnTape() + "\t" + RobotMap.innerRightSensor.isOnTape() + "\t" + RobotMap.outerRightSensor.isOnTape());
	}
}