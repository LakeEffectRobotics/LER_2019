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
    requires(Robot.intakeArm);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    double currentPos = Robot.intakeArm.getTargetPosition();
    
    if(currentPos > IntakeArm.POSITION_UP) // If it's past up
      Robot.intakeArm.setTargetPosition(IntakeArm.POSITION_UP);
    else if(currentPos > IntakeArm.POSITION_MID) // If it's between half and up
      Robot.intakeArm.setTargetPosition(IntakeArm.POSITION_DOWN);
    else if(currentPos < IntakeArm.POSITION_MID) // If it's between half and down
      Robot.intakeArm.setTargetPosition(IntakeArm.POSITION_UP);
    
  }

}
