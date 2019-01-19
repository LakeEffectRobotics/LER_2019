package frc.robot.commands.instant;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ToggleShawnDriveCommand extends InstantCommand {
    public ToggleShawnDriveCommand() {
        super();
        requires(Robot.drivetrain);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.drivetrain.shawn_drive_active = !Robot.drivetrain.shawn_drive_active;
    }
}
