/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.revrobotics.CANSparkMax;


public class Drivetrain extends Subsystem {
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveCommand());
	}
	
	public void drive(double l, double r) {
		//Other motors are followers
		System.out.println(l+"\t"+r);
		RobotMap.leftDriveSpark1.set(-l);
		RobotMap.rightDriveSpark1.set(r);
	}
	
}
