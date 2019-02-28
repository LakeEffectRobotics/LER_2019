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
import frc.robot.commands.IntakeArmCommand;

/**
 * Add your docs here.
 */
public class IntakeArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //TODO: Set positions
  public static final double POSITION_MAX = 3967;
  public static final double POSITION_UP = 3620;
  public static final double POSITION_DOWN = 2577;
  public static final double POSITION_MID = (POSITION_UP+POSITION_DOWN)/2;
  
  public double targetPosition;
  public static final double ACCEL = 1;
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new IntakeArmCommand());
  }

  public void setTargetPosition(double position){
    if(position > POSITION_MAX) position=POSITION_MAX;
    if(position < POSITION_DOWN) position=POSITION_DOWN;
    targetPosition = position; 
  }

  public void drive(){
    double speed = Math.max(Math.min((targetPosition - getPosition())/1000, 1), -1)/2;
    if(targetPosition==POSITION_DOWN && getPosition()<POSITION_MID)
      speed /= 10;
    RobotMap.intakeArmTalon.set(ControlMode.PercentOutput, speed);
    // System.out.println("Speed: "+(targetPosition - RobotMap.intakePot.getValue())/1000+"\tTarget: "+targetPosition+"\tCurrent"+RobotMap.intakePot.getValue());

  }

  public double getPosition(){
    return(RobotMap.intakeArmTalon.getSelectedSensorPosition());
  }

  public double getTargetPosition(){
    return(targetPosition);
  }

  
}
