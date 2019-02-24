/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.instant;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeArm;

/**
 * Add your docs here.
 */
public class ToggleIntakeArm extends InstantCommand {
  /**
   * Add your docs here.
   */
  public ToggleIntakeArm() {
    super();
    // Use requires() here to declare subsystem dependencies
    requires(robot.IntakeArm);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    
    }
  }

}
