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
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //TODO: Set heights
  final double GROUND_HEIGHT = 0;
  final double MAX_HEIGHT = 0;


  double targetHeight = GROUND_HEIGHT;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand();
  }

  public double getHeight(){
    return(RobotMap.elevatorTalon.getSelectedSensorPosition());
  }

  public double getTargetHeight(){
    return(targetHeight);
  }

  public void setTargetHeight(double target){
    if(target > MAX_HEIGHT) target=MAX_HEIGHT;
    if(target < GROUND_HEIGHT) target = GROUND_HEIGHT;
    targetHeight = target;    

    RobotMap.elevatorTalon.set(ControlMode.Position, target);
  }
}
