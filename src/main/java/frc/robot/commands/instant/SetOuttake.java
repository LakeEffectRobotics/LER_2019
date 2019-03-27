/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands.instant;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Outtake;

/**
 * Add your docs here.
 */
public class SetOuttake extends InstantCommand {

  int side = 5;
  int position;
  public static final int SIDE_AUTO = 5;
  public static final int SIDE_BOTH = 6;

  public static int AUTO_OUT = -1;
  public static int AUTO_IN = -2;
  /**
   * Add your docs here.
   */
  public SetOuttake(int s, int pos) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // requires(Robot.outtake);
    side = s;
    position=pos;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    System.out.println("Outtake: "+side);
    if(side == SIDE_BOTH){
      if(position == AUTO_IN){
        Robot.outtake.setSide(Outtake.SIDE_LEFT, Outtake.L_IN);
        Robot.outtake.setSide(Outtake.SIDE_RIGHT, Outtake.R_IN);
      }
      return;
    }
    if(side == SIDE_AUTO){
      System.out.println("AUTO");
      double distance = (RobotMap.leftDriveSpark2.getEncoder().getPosition()+RobotMap.rightDriveSpark2.getEncoder().getPosition())/2.0;
      if(Robot.outtake.lastSide == Outtake.SIDE_LEFT && distance < Outtake.MAX_DIST){
        side = Outtake.SIDE_LEFT;
        System.out.println("LEFT");
        if(position == AUTO_OUT) position = Outtake.L_OUT;
        if(position == AUTO_IN) position = Outtake.L_IN;
      }
      else if(Robot.outtake.lastSide == Outtake.SIDE_RIGHT && distance < Outtake.MAX_DIST){
        side=Outtake.SIDE_RIGHT;
        System.out.println("RIGHT");
        if(position == AUTO_OUT) position = Outtake.R_OUT;
        if(position == AUTO_IN) position = Outtake.R_IN;
      }
      else{
        System.out.println("NOPE");
        return;
      }
    }
    // if(position == Outtake.L_OUT){
    //   Robot.outtake.setSide(Outtake.SIDE_LEFT,Outtake.L_OUT);
    //   Robot.outtake.setSide(Outtake.SIDE_RIGHT,Outtake.R_IN);
    // }
    // if(position == Outtake.R_OUT){
    //   Robot.outtake.setSide(Outtake.SIDE_LEFT,Outtake.L_IN);
    //   Robot.outtake.setSide(Outtake.SIDE_RIGHT,Outtake.R_OUT);
    // }
    // else{
    Robot.outtake.setSide(side, position);
    // }
  }

}
