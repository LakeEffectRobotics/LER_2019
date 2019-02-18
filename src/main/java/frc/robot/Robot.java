/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Lift;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	public static final Drivetrain drivetrain = new Drivetrain();
	public static final Gyro gyro = new Gyro();
	public static final CargoIntake cargoIntake = new CargoIntake();
	public static final Lift lift = new Lift();
	public static final Climber climber = new Climber();
	public static OI oi;
	
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

	@Override
	public void testPeriodic() {
		////
	}
}
