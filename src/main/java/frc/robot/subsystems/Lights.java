/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.awt.Color;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Lights extends Subsystem {

  public enum Colour {
    RED, BLUE, PURPLE, OFF
  }

  public static int LEFT = 0;
  public static int RIGHT = 1;

  @Override
  public void initDefaultCommand() {
  }

  public void setColour(int side, Colour c){
    Relay red;
    Relay blue;

    if(side==LEFT){
      red=RobotMap.leftRed;
      blue=RobotMap.leftBlue;
    } else if(side==RIGHT){
      red=RobotMap.rightRed;
      blue=RobotMap.rightBlue;
    } else {
      return;
    }

    switch(c){
      case RED:
        red.set(Value.kOn);
        blue.set(Value.kOff);
      case BLUE:
        red.set(Value.kOff);
        blue.set(Value.kOn);
      case PURPLE:
        red.set(Value.kOn);
        blue.set(Value.kOn);
      case OFF:
        red.set(Value.kOff);
        blue.set(Value.kOff);
    }
  }

  public void setBoth(Colour c){
    setColour(LEFT, c);
    setColour(RIGHT, c);
  }
}
