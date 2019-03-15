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
  final static double MAX_DRIVE_DISTANCE = 18;    //  If the "B" sensors fail to detect tape after initially driving this distance, the command will simply finish.
  final static double SENSOR_SPACING = 12;        //  The distance between the "A" and "B" sensors (and the distance that must be driven once the "B" sensors detect the tape). 
  final static double MIN_SPEED = 0.2;
  final static double FINISHED_THRESHOLD = 4;     //  If the "A" sensors fail to detect tape after driving this distance past SENSOR_SPACING, the command will simply finish.

  double speed;
  boolean isLeftSide;
  boolean[] tapeDetected = new boolean[3];        //  tapeDetected[0] states if a "B" sensor has detected the tape. tapeDetected[1] states if the first corresponding "A" sensor has detected the tape. tapeDetected[2] states if the second corresponding "A" sensor has detected the tape. 
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
    Robot.drivetrain.resetEncoderPositions();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double averageEncoderPosition = -Robot.drivetrain.getAverageEncoderPosition();  //  Positions and velocities are inverted here because the robot is to be driven in reverse (in the direction of the hatch panel mechanism). 
    if (!tapeDetected[0]) {
      if (averageEncoderPosition < MAX_DRIVE_DISTANCE) {
        Robot.drivetrain.drive(-speed, -speed);
        if (RobotMap.tapeSensorB1.isOnTape()) {
          isLeftSide = true;
          tapeDetected[0] = true;
          Robot.drivetrain.resetEncoderPositions();
        } else if (RobotMap.tapeSensorB2.isOnTape()) {
          isLeftSide = false;
          tapeDetected[0] = true;
          Robot.drivetrain.resetEncoderPositions();
        }
      } else {
        finished = true;
      }
    } else {
      double distanceLeft = SENSOR_SPACING - averageEncoderPosition;
      double output = 0;
      
      if (distanceLeft > SENSOR_SPACING) {
    		output = speed;
    	} else if (distanceLeft < -SENSOR_SPACING) {
    		output = -speed;
      } else {
        output = speed * (distanceLeft / SENSOR_SPACING);
        output = Tools.setAbsoluteMinimum(output, MIN_SPEED);
      }
      Robot.drivetrain.drive(-speed, -speed);

      if (robotOnTape() || distanceLeft < -FINISHED_THRESHOLD) {
        finished = true;
      }
    }
  }
  //  TODO: Add some LED code in here to let drive team know if the line-up has succeeded/failed

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

  protected boolean robotOnTape() {
    if (isLeftSide) {
      if (RobotMap.tapeSensorA1.isOnTape()) {
        tapeDetected[1] = true;
      }
      if (RobotMap.tapeSensorA2.isOnTape()) {
        tapeDetected[2] = true;
      }
    } else {
      if (RobotMap.tapeSensorA3.isOnTape()) {
        tapeDetected[1] = true;
      }
      if (RobotMap.tapeSensorA4.isOnTape()) {
        tapeDetected[2] = true;
      }
    }
    return (tapeDetected[1] && tapeDetected[2]);
  }
}
