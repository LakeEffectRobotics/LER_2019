/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.instant;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Outtake;

/**
 * Add your docs here.
 */
public class ToggleOuttakeCommand extends InstantCommand {
  /**
   * Add your docs here.
   */

   private Outtake.Side side;

  public ToggleOuttakeCommand(Outtake.Side s) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    side = s;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(Robot.outtake.getPosition(side)==Outtake.Position.OUT){
      Robot.outtake.extend(side);
    } else {
      Robot.outtake.retract(side);
    }
  }

}
