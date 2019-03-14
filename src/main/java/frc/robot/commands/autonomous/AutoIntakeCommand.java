/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.IntakeArm;

public class AutoIntakeCommand extends Command {
  public AutoIntakeCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.intakeArm);
    requires(Robot.intakeRoller);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.intakeArm.setTargetPosition(IntakeArm.POSITION_DOWN);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.intakeRoller.spin(0.5);
    if(RobotMap.intakeLimitSwitch.get()){
      Robot.intakeArm.setTargetPosition(IntakeArm.POSITION_UP);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.intakeArm.getTargetPosition()==IntakeArm.POSITION_UP && !RobotMap.intakeLimitSwitch.get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.intakeRoller.spin(0);
    Robot.intakeArm.setTargetPosition(IntakeArm.POSITION_UP);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
