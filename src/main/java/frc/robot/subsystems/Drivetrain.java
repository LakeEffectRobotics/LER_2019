/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Drivetrain extends Subsystem {
	public boolean shawnDriveIsActive = false;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveCommand());
	}
	
	public void drive(double l, double r) {
		//Other motors are followers
		
		if (!shawnDriveIsActive) {
			//  Standard drive
			RobotMap.leftDriveSpark1.set(l);
			RobotMap.rightDriveSpark1.set(-r);  //  r is inverted because the left and right motors are oriented in opposite directions
		} else {
			//  Drive with robot's front/back switched
			RobotMap.leftDriveSpark1.set(-r);
			RobotMap.rightDriveSpark1.set(l);
		}
	}
}
