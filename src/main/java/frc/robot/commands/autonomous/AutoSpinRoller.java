/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class AutoSpinRoller extends InstantCommand {

  double speed;

  public AutoSpinRoller(double speed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.intakeRoller);
    this.speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.intakeRollerTalon.set(ControlMode.PercentOutput, speed);
  }

}
