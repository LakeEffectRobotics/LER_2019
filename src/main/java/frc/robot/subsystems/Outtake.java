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

  public static double L_IN = 0;
  public static double L_OUT = 0;
  public static double R_IN = 0;
  public static double R_OUT = 0;

  public static int RIGHT = 0;
  public static int LEFT = 1;

  double targetL = 0;
  double targetR = 0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void toggleSide(int side){
    if(side == LEFT){
      if(targetL == L_IN)
        targetL = L_OUT;
      else
        targetL = L_IN;
      RobotMap.leftServo.set(targetL);
    }

    if(side == LEFT){
      if(targetL == R_IN)
        targetL = R_OUT;
      else
        targetL = R_IN;
      RobotMap.rightServo.set(targetR);
    }
  }

  public void setSide(int side, double target){
    //TODO: Add safeguard
    
    if(side == LEFT){
      targetL = target;
      RobotMap.leftServo.set(targetL);
    }
    
    if(side == LEFT){
      targetR = target;
      RobotMap.rightServo.set(targetR);
    }
  }

}
