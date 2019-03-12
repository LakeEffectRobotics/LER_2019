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

public class VisionDriveCommand extends Command {

  double lSpeed, rSpeed;
  int offset = 0;
  /**
   * Amount of bits in each of the last 5 messages, used to check for data
   * validity
   */
  int[] collector = { 0, 0, 2, 0, 0 };

  /** The offset where speed=1, inversely proportinal to power */
  final double MAX_DIFF = 20.0;
  final double MULTIPLIER = 0.25;

  public VisionDriveCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    int available =0;// RobotMap.jevoisSerial.getBytesReceived();
    // Update collector
    int sum = available;
    for (int i = 0; i < collector.length - 1; i++) {
      collector[i] = collector[i] + 1;
      sum += collector[i];
    }
    collector[collector.length - 1] = available;

    // If there have been <2 bits in the last 5 messages, there's an issue
    if (sum < 2) {
      offset = 0;
    }

    // Parse new offset
    if (available > 0) {
      String[] in = {""};//RobotMap.jevoisSerial.readString().split("\n");
      System.out.println(in[0]);
      String t = in[in.length - 1];
      if (t.length() > 1) {
        // Remove the trailing whitespace
        t = t.substring(0, t.length() - 1);
        offset = Integer.parseInt(t);
      }
    }

    offset = (int) fitToRange(offset, -MAX_DIFF, MAX_DIFF);

    lSpeed = Math.pow(offset / (MAX_DIFF), 3) * MULTIPLIER;
    lSpeed += 0.5 * Robot.oi.l_joy.getY();

    rSpeed = -Math.pow(offset / (MAX_DIFF), 3) * MULTIPLIER;
    rSpeed += 0.5 * Robot.oi.l_joy.getY();

    // System.out.println(available+"\t"+offset+"\t"+lSpeed+"\t"+rSpeed);

    Robot.drivetrain.drive(fitToRange(-lSpeed, -1.0, 1.0), fitToRange(rSpeed, -1.0, 1.0));
  }

  public double fitToRange(double value, double min, double max) {
    value = value < min ? min : value;
    value = value > max ? max : value;
    return value;
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
