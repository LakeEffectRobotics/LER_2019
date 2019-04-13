package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Robot.AutonomousStartPosition;
import frc.robot.commands.instant.SetElevatorHeightCommand;
import frc.robot.commands.instant.SetOuttake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Outtake;

/**
 * Main autonomous command group for auto<p>
 * There are 3 'Points' A,B, and C. <br>
 * <ul><b>A</b> is directly in front of the closest switch<br>
 * <b>B</b> is between the closest switch and scale, just a few feet ahead of A<br>
 * <b>C</b> is between the far switch and scale, on the opposite side of the field<p>
 * </ul>
 * At each point the robot either continues to the next or breaks off to score.
 * 
 *@author Ewan
 *
 *
 */
public class AutoCommandGroup extends CommandGroup {
	final double SPEED = 0.5;
	final double D_OFF_LEVEL1 = 10; 
	final double D_OFF_LEVEL2 = 20; 
	final double D_SCORE_HATCH = 1; 

	public AutoCommandGroup() {
		Robot.AutonomousTarget target = Robot.autonomous_target_chooser.getSelected();
		Robot.AutonomousStartPosition startPosition = Robot.autonomous_position_chooser.getSelected();
		Robot.AutonomousGamepiece gamepiece = Robot.autonomous_gamepiece_chooser.getSelected();

		/*********************************************************************
		 * 
		 * First get to the right place depending on start position and target
		 * 
		 *********************************************************************/

		switch (target) {
		case CARGO_FRONT:
			if (startPosition == AutonomousStartPosition.LEFT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL2));
			} else if (startPosition == AutonomousStartPosition.RIGHT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL1));
			} else if (startPosition == AutonomousStartPosition.MID_LEFT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL1));
			} else {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL2));
			}
			break;
		case CARGO_1:
		case CARGO_2:
		case CARGO_3:
			if (startPosition == AutonomousStartPosition.LEFT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL2));
			} else if (startPosition == AutonomousStartPosition.RIGHT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL1));
			} else if (startPosition == AutonomousStartPosition.MID_LEFT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL1));
			} else {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL2));
			}
			break;
		case ROCKET_NEAR:
			if (startPosition == AutonomousStartPosition.LEFT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL2));
			} else if (startPosition == AutonomousStartPosition.RIGHT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL1));
			} else if (startPosition == AutonomousStartPosition.MID_LEFT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL1));
			} else {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL2));
			}
			break;
		case ROCKET_FAR:
			if (startPosition == AutonomousStartPosition.LEFT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL2));
			} else if (startPosition == AutonomousStartPosition.RIGHT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL1));
			} else if (startPosition == AutonomousStartPosition.MID_LEFT) {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL1));
			} else {
				addSequential(new AutoGyroDriveCommand(SPEED, D_OFF_LEVEL2));
			}
			break;
		default:
			break;
		}
		/*********************************************************************
		 * 
		 * Line up with vision target or white line
		 * 
		 *********************************************************************/
		if (gamepiece==Robot.AutonomousGamepiece.HATCH)
		{
			addSequential(new VisionDriveCommand());
		} else if (gamepiece==Robot.AutonomousGamepiece.CARGO)
		{
			addSequential(new TapeAlignCommand());
		} 

		/*********************************************************************
		 * 
		 * Score gamepiece (either cargo or hatch)
		 * 
		 *********************************************************************/
		if (gamepiece==Robot.AutonomousGamepiece.HATCH || target==Robot.AutonomousTarget.ROCKET_NEAR || target==Robot.AutonomousTarget.ROCKET_FAR)
		{
			addSequential(new SetElevatorHeightCommand(Elevator.LOW_HEIGHT));
		} else {
			addSequential(new SetElevatorHeightCommand(Elevator.CARGO_SHIP_HEIGHT));
		}
		switch (gamepiece) {
			case CARGO:
				if (startPosition == AutonomousStartPosition.LEFT || startPosition == AutonomousStartPosition.MID_LEFT)
				{
					addSequential(new SetOuttake(Outtake.SIDE_LEFT,Outtake.L_OUT));
				} else {
					addSequential(new SetOuttake(Outtake.SIDE_RIGHT,Outtake.R_OUT));
				}
				break;
			case HATCH:
				addSequential(new AutoGyroDriveCommand(SPEED/2, D_SCORE_HATCH));
				addSequential(new SetElevatorHeightCommand(Elevator.LOW_HEIGHT-Elevator.BUMP_DOWN));
				addSequential(new AutoGyroDriveCommand(SPEED/2, -D_SCORE_HATCH));
				break;
			default:
		}
		//addSequential(new Stop());
	}
}
