/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Tools;

public class TapeAlignCommand extends Command {
  //  TODO: Update these values
  final static double MAX_DRIVE_DISTANCE = 15;
  final static double SENSOR_SPACING = 12;
  final static double MIN_SPEED = 0.2;
  final static double FINISHED_TOLERANCE = 0.5;

  double speed;
  double[] initialEncoderPositions;
  double averageEncoderPosition;
  double distanceLeft;
  boolean isLeftSide;
  boolean tapeDetected = false;
  boolean finished = false;

  public TapeAlignCommand(double speed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
    this.speed = speed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initialEncoderPositions = Robot.drivetrain.getInitialEncoderPositions();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putBoolean("Detected Tape", tapeDetected);
    averageEncoderPosition = Tools.rotationsToInches(Robot.drivetrain.getAverageEncoderPosition(initialEncoderPositions));
    if (!tapeDetected) {
      if (averageEncoderPosition < MAX_DRIVE_DISTANCE) {
        Robot.drivetrain.drive(speed, speed);
      }
      if (RobotMap.leftTapeSensor1.isOnTape()) {
        isLeftSide = true;
        tapeDetected = true;
        initialEncoderPositions = Robot.drivetrain.getInitialEncoderPositions();
      } else if (RobotMap.rightTapeSensor1.isOnTape()) {
        isLeftSide = false;
        tapeDetected = true;
        initialEncoderPositions = Robot.drivetrain.getInitialEncoderPositions();
      } else if (averageEncoderPosition >= MAX_DRIVE_DISTANCE) {
        finished = true;
      }
    } else {
      distanceLeft = SENSOR_SPACING - averageEncoderPosition;
      speed *= (distanceLeft / SENSOR_SPACING);
      speed = Tools.forceMaximum(speed, MIN_SPEED);
      Robot.drivetrain.drive(speed, speed);
      // if (centeredOnTape() || -distanceLeft >= FINISHED_TOLERANCE) {
      //   finished = true;
      // }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
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

  // protected boolean centeredOnTape() {
  //   // return (isLeftSide && RobotMap.leftTapeSensor2.isOnTape()) || (!isLeftSide && RobotMap.rightTapeSensor2.isOnTape());
  // }
}
