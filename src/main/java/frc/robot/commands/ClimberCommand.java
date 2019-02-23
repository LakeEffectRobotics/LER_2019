/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class ClimberCommand extends Command {
  public ClimberCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    int targetPosition = RobotMap.climberTalon1.getTargetPosition();
    /*
    TODO: Create controls similar to those on the 2018 lift
    --------------------------------------------------------------
    if (Robot.oi.scale_height_button.get()) {
      target_position = Lift.SCALE_HEIGHT;
    }
    else if (Robot.oi.switch_height_button.get()) {
      target_position = Lift.SWITCH_HEIGHT;
    }
    else if (Robot.oi.ground_height_button.get()) {
      target_position = Lift.GROUND_HEIGHT;
      }
    target_position += Robot.oi.xbox.getJoyRightY() * MULTIPLIER;
    --------------------------------------------------------------
    public final static int UP_POSITION = 2708;
    public final static int DOWN_POSITION = 2708;
    */
    RobotMap.climberTalon1.setTargetPosition(targetPosition);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
