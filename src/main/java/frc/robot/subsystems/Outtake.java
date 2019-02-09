/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Outtake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum Position{		  
    IN(DoubleSolenoid.Value.kReverse),
    OUT(DoubleSolenoid.Value.kForward),
    OFF(DoubleSolenoid.Value.kOff);

    private DoubleSolenoid.Value value;    

    private Position(DoubleSolenoid.Value value) {
      this.value = value;
    }

    public DoubleSolenoid.Value getValue() {
      return value;
    }
  };

  public enum Side {
    LEFT,RIGHT
  }

  Position lPosition = Position.IN;
  Position rPosition = Position.IN;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setPosition(Position p, Side s){
    switch(s){
      case LEFT:
        RobotMap.leftFlipper.set(p.getValue());
        lPosition = p;
      case RIGHT:
        RobotMap.rightFlipper.set(p.getValue());
        rPosition = p;
    }
  } 

  public void extend(Side s){
    setPosition(Position.OUT, s);
  }

  public void retract(Side s){
    setPosition(Position.IN, s);
  }

  public void retractAll(){
    setPosition(Position.IN, Side.LEFT);
    setPosition(Position.IN, Side.RIGHT);
  }
  
  public void extendAll(){
    setPosition(Position.OUT, Side.LEFT);
    setPosition(Position.OUT, Side.RIGHT);
  }


  public Position getPosition(Side s){
    switch(s){
      case LEFT:
        return lPosition;
      case RIGHT:
       return rPosition;
      default:
        return Position.OFF;
    }
  }

}
