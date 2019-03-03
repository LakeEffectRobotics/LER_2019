/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class HoldOuttake extends Command {

  int side;
  double position;

  /**
   * Add your docs here.
   */
  public HoldOuttake(int s, double pos) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.outtake);
    side = s;
    position=pos;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.outtake.setSide(side,position);
    // System.out.println("OUTTAKE "+side);
  }

  @Override
  protected boolean isFinished(){
    return false;
  }

  @Override
  protected void end(){
    //Robot.outtake.toggleSide(side);
  }

}
