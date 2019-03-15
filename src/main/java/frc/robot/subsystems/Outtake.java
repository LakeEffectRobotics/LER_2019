/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Outtake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static double L_GRIP = 0.6;
  public static double L_IN = 0.51;
  public static double L_OUT = 0;
  public static double R_GRIP = 0.5;
  public static double R_IN = 0.41;
  public static double R_OUT = 1;

  public static int SIDE_RIGHT = 1;
  public static int SIDE_LEFT = 0;

  double targetL = 0;
  double targetR = 0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void toggleSide(int side){
    // System.out.println("TOGGLE");
    if(side == SIDE_LEFT){
      if(targetL == L_IN)
        targetL = L_OUT;
      else
        targetL = L_IN;
      RobotMap.leftServo.set(targetL);
      // System.out.println("LEFT "+targetL);
    }

    if(side == SIDE_RIGHT){
      if(targetR == R_IN)
        targetR = R_OUT;
      else
        targetR = R_IN;
      RobotMap.rightServo.set(targetR);
      // System.out.println("RIGHT "+targetR);
    }
  }

  public void setSide(int side, double target){
    //TODO: Add safeguard
    
    if(side == SIDE_LEFT){
      targetL = target;
      RobotMap.leftServo.set(targetL);
      // System.out.println("LEFT"+targetL);
    }
    
    if(side == SIDE_RIGHT){
      targetR = target;
      RobotMap.rightServo.set(targetR);
      // System.out.println("RIGHT"+targetR);
    }
  }

}
