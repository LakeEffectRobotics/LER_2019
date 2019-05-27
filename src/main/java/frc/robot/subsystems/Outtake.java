/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.OuttakeCommand;

/**
 * Add your docs here.
 */
public class Outtake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /*
   * public static double L_GRIP = 0.6; Original outtake public static double L_IN
   * = 0.51; public static double L_OUT = 0; public static double R_GRIP = 0.5;
   * public static double R_IN = 0.41; public static double R_OUT = 1;
   */
  /* Commented out by Mr. Wood - these are values for the twin before replacing pots???
  public static final int L_GRIP = 200;
  public static final int L_IN = 226;
  public static final int L_OUT = 500;
  public static final int R_GRIP = -220;
  public static final int R_IN = -248;
  public static final int R_OUT = -410;*/

  public static final int L_GRIP = 685;
  public static final int L_IN = 660;//-555;
  public static final int L_OUT = 460;
  public static final int R_GRIP = -930;//855;
  public static final int R_IN = -946;//803;
  public static final int R_OUT = -984; //635;

  public boolean L_DISABLED = false;
  public boolean R_DISABLED = false;

  public static final int SIDE_RIGHT = 1;
  public static final int SIDE_LEFT = 0;
  public static final int SIDE_NONE = -1;

  /***The maxium distance from the line, in inches */
  public static final int MAX_DIST = 4;

  int lTarget = -1;

  int rTarget = -1;

  public int lastSide = SIDE_NONE;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new OuttakeCommand());
  }

  public void setSide(int side, int t) {
    int target = (int) t;

    if (side == SIDE_LEFT) {
      lTarget = target;
      // if(Robot.outtake.L_DISABLED){
      //   return;
      // }
    }

    if (side == SIDE_RIGHT) {
      rTarget = target;
    }
    RobotMap.leftOuttakeTalon.set(ControlMode.Position,lTarget);
    RobotMap.rightOuttakeTalon.set(ControlMode.Position,rTarget);

    if(Robot.oi.rightIn.get() || Robot.oi.rightOut.get()) R_DISABLED = true;
    if(Robot.oi.leftIn.get()  || Robot.oi.leftOut.get()) L_DISABLED = true;

    if(L_DISABLED){
      int lVal = 0;
      lVal += Robot.oi.leftIn.get()?1:0;
      lVal -= Robot.oi.leftOut.get()?1:0;
      RobotMap.leftOuttakeTalon.set(ControlMode.PercentOutput, lVal/2);
    }
    if(R_DISABLED){
      int rVal = 0;
      rVal += Robot.oi.rightIn.get()?1:0;
      rVal -= Robot.oi.rightOut.get()?1:0;
      RobotMap.rightOuttakeTalon.set(ControlMode.PercentOutput, rVal/2);
    }
  }

}
