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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class Robot extends TimedRobot {

	public static final Drivetrain drivetrain = new Drivetrain();
	public static final Gyro gyro = new Gyro();
	public static OI oi;

	public static final IntakeArm intakeArm = new IntakeArm();
	public static final IntakeRoller intakeRoller = new IntakeRoller();
	public static final Elevator elevator = new Elevator();
	
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
//		if (autonomous_command_group != null) {
//			autonomous_command_group.cancel();
//		}
		enabledInit();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		robotPeriodic();
	}

	TalonSRX t = new TalonSRX(7);
	VictorSPX v = new VictorSPX(13);
	public void testInit() {
		v.follow(t);
	}

	@Override
	public void testPeriodic() {
		t.set(ControlMode.PercentOutput, Robot.oi.l_joy.getY()/2);
	}
}
