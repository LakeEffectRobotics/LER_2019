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
import frc.robot.Tools;

public class AutoGyroDriveCommand extends Command {
    private double speed;
    private double distance;
    private boolean finished = false;

    public AutoGyroDriveCommand(double speed, double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drivetrain);
        this.speed = speed;
        this.distance = distance;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.drivetrain.resetEncoderPositions();
        Robot.gyro.resetAngle();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Object[] auto_drive_output = Robot.drivetrain.getAutoDriveOutput(speed, distance, timeSinceInitialized());
        double[] straight_gyro_output = Robot.gyro.getStraightOutput((double) auto_drive_output[0], (double) auto_drive_output[1]);
        Robot.drivetrain.drive(straight_gyro_output[0], straight_gyro_output[1]);
        finished = (boolean) auto_drive_output[2];
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
}
