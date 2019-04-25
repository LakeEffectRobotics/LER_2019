/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Tools;
import frc.robot.subsystems.Elevator;

public class AuotnomousPlaceHatchCommand extends Command {

  double lSpeed, rSpeed;

  final double MAX_DIFF = 20.0;
  final double kP = 0.075;

  int maxDist = -1;
  public AuotnomousPlaceHatchCommand(int maxDist) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivetrain);
    requires(Robot.elevator);
    this.maxDist = maxDist;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.visionOffset = (int) Tools.fitToRange(Robot.visionOffset, -MAX_DIFF, MAX_DIFF);

    int error = maxDist - (int) Robot.drivetrain.getAverageEncoderPosition();

    lSpeed = Math.pow(Robot.visionOffset / (MAX_DIFF), 3) * kP;
    lSpeed += (error*error)/1000+0.2;

    rSpeed = -Math.pow(Robot.visionOffset / (MAX_DIFF), 3) * kP;
    rSpeed += (error*error)/1000+0.2;

    //System.out.println(available+"\t"+offset+"\t"+lSpeed+"\t"+rSpeed);

    Robot.drivetrain.drive(Tools.fitToRange(-lSpeed, -1.0, 1.0), Tools.fitToRange(-rSpeed, -1.0, 1.0));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Robot.drivetrain.getAverageEncoderPosition() > maxDist); //TODO: or limit switch pressed;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.setOffset(Elevator.BUMP_DOWN);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
