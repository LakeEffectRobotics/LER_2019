/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Outtake;

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
	
	public void robotPeriodic() {
		//Called periodically, use to interface with dashboard
	}

	public void enabledInit() {
		RobotMap.gyro.reset();
	}

	@Override
	public void robotInit() {
		oi = new OI();
		oi.init();
		RobotMap.init();
		gyro.calibrate();
		lights.setBoth(Lights.Colour.PURPLE);
		//Setup dashboard
	}

	@Override
	public void disabledInit() {
		////
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		robotPeriodic();
		System.out.println(RobotMap.elevatorSpark1.getEncoder().getPosition());
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

		Robot.intakeArm.setTargetPosition(IntakeArm.POSITION_DOWN);

		enabledInit();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		robotPeriodic();
		// System.out.println(Robot.elevator.getTargetHeight()+"\t"+RobotMap.elevatorSpark1.getEncoder().getPosition());
	}

	//3653,3967
	Servo s = new Servo(0);
	double d = 0;

	// AnalogInput a = new AnalogInput(0);

	public void testInit() {
	}

	@Override
	public void testPeriodic() {
		d += Robot.oi.xbox.getJoyLeftY()/200;
		System.out.println(d);
		s.set(d);
		// System.out.println(a.getValue());
	}
}

