/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

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

  public static final int L_GRIP = 0;
  public static final int L_IN = 0;
  public static final int L_OUT = 0;
  public static final int R_GRIP = 0;
  public static final int R_IN = 0;
  public static final int R_OUT = 0;

  public static final int SIDE_RIGHT = 1;
  public static final int SIDE_LEFT = 0;
  public static final int SIDE_NONE = -1;

  final static double P = 0.1;

  int targetL = 0;
  int targetR = 0;

  public int lastSide = SIDE_NONE;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setSide(int side, int target) {
    if (side == SIDE_LEFT) {
      targetL = target;
    }

    if (side == SIDE_RIGHT) {
      targetR = target;
    }
  }

  public void drive() {
    int lDelta = RobotMap.leftOuttakeCounter.get() - targetL;
    double lSpeed = lDelta * P;
    RobotMap.leftOuttakeCounter.setReverseDirection(lDelta<0);
    
    int rDelta = RobotMap.rightOuttakeCounter.get() - targetR;
    double rSpeed = lDelta * P;
    RobotMap.rightOuttakeCounter.setReverseDirection(rDelta<0);

    RobotMap.leftOuttakeTalon.set(ControlMode.PercentOutput, lSpeed);
    RobotMap.rightOuttakeTalon.set(ControlMode.PercentOutput, rSpeed);
  }

}
