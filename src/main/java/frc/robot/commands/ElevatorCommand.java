/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {

  final double DEADZONE = 0.2;

  public ElevatorCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //F greg
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Manual control using left stick
    if(Math.abs(Robot.oi.xbox.getJoyLeftY()) > DEADZONE){
      Robot.elevator.setTargetHeight(Robot.elevator.getTargetHeight()+Robot.oi.xbox.getJoyLeftY()/2, 0, "Joy");
    }
    //Offset for grabbing+releasing hatches
    if(Robot.oi.xbox.getTriggerRight() > DEADZONE){
      Robot.elevator.setOffset(Robot.oi.xbox.getTriggerRight()*Elevator.HATCH_RELEASE_OFFSET);
    } else {
      Robot.elevator.setOffset(0);
    }

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
