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
public class IntakeArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //TODO: Set positions
  public static final double POSITION_MAX = 0;
  public static final double POSITION_UP = 0;
  public static final double POSITION_DOWN = 0;
  public static final double POSITION_MID = (POSITION_UP+POSITION_DOWN)/2;
  
  public double targetPosition;
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setTargetPosition(double position){

    if(position > POSITION_MAX) position=POSITION_MAX;
    if(position < POSITION_DOWN) position=POSITION_DOWN;
    targetPosition = position; 

    RobotMap.intakeArmTalon.set(ControlMode.Position, targetPosition);
  }

  public double getTargetPosition(){
    return(RobotMap.intakeArmTalon.getSelectedSensorPosition());
  }

  public double getPosition(){
    return(targetPosition);
  }

  
}
