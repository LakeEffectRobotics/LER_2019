/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Outtake;

/**
 * Add your docs here.
 */
public class HoldOuttake extends InstantCommand {

  int side = 5;
  double position;
  String src;
  public static final int SIDE_AUTO = 5;
  public static final int SIDE_BOTH = 6;

  public static double AUTO_OUT = -1;
  public static double AUTO_IN = -2;
  /**
   * Add your docs here.
   */
  public HoldOuttake(int s, double pos, String src) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.outtake);
    side = s;
    position=pos;
    this.src = src;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    System.out.println("Outtake: "+src+","+side);
    if(side == SIDE_BOTH){
      if(position == AUTO_IN){
        Robot.outtake.setSide(Outtake.SIDE_LEFT, Outtake.L_IN);
        Robot.outtake.setSide(Outtake.SIDE_RIGHT, Outtake.R_IN);
      }
      return;
    }
    System.out.println("Still going");
    if(side == SIDE_AUTO){
      System.out.println("AUTO");
      if(RobotMap.outerRightSensor.isOnTape()){
        side = Outtake.SIDE_LEFT;
        System.out.println("LEFT");
        if(position == AUTO_OUT) position = Outtake.L_OUT;
        if(position == AUTO_IN) position = Outtake.L_IN;
      }
      else if(RobotMap.outerLeftSensor.isOnTape()){
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
    Robot.outtake.setSide(side,position);
    // System.out.println("OUTTAKE "+side);
  }

}
