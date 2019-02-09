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
    IN,LEFT,RIGHT,BOTH
  };

  Position currentPos = Position.IN;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setPostition(Position p){
    if(currentPos == p) return;

    switch(p){
      case IN:
        RobotMap.leftFlipper.set(DoubleSolenoid.Value.kReverse);
        RobotMap.rightFlipper.set(DoubleSolenoid.Value.kReverse);
      case LEFT:
        RobotMap.leftFlipper.set(DoubleSolenoid.Value.kForward);
        RobotMap.rightFlipper.set(DoubleSolenoid.Value.kReverse);
      case RIGHT:
        RobotMap.leftFlipper.set(DoubleSolenoid.Value.kReverse);
        RobotMap.rightFlipper.set(DoubleSolenoid.Value.kForward);
      case BOTH:
        RobotMap.leftFlipper.set(DoubleSolenoid.Value.kForward);
        RobotMap.rightFlipper.set(DoubleSolenoid.Value.kForward);
    }

    currentPos = p;
  } 

  public void retract(){
    setPostition(Position.IN);
  }

}
