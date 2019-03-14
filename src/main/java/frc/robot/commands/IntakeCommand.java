/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  public IntakeCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.intakeArm.setTargetPosition(IntakeArm.POSITION_UP);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Math.abs(Robot.oi.xbox.getJoyRightY()) > 0.2){
      Robot.intake.setTargetPosition(Robot.intake.getTargetPosition()+Robot.oi.xbox.getJoyRightY());
    }
    //Retract the arm for defence
    if(Robot.oi.xbox.getDpadUp()){
      Robot.intake.setTargetPosition(Intake.POSITION_MAX);
    }

    if(RobotMap.intakeLimitSwitch.get() && Robot.oi.xbox.getTriggerLeft() > 0.1)
      Robot.intake.setTargetPosition(Intake.POSITION_UP);

    Robot.intake.spin(Robot.oi.xbox.getTriggerLeft());
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
