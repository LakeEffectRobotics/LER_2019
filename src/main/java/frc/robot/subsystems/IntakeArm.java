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
  // Competition bot:
  /*public static final double POSITION_MAX = -220;
  public static final double POSITION_UP = -120;
  public static final double POSITION_DOWN = -5;
  public static final double POSITION_MID = (POSITION_UP+POSITION_DOWN)/2;*/

  // Twin Bot:
  public static final double POSITION_MAX = -205;
  public static final double POSITION_UP = -138;
  public static final double POSITION_DOWN = -4;
  public static final double POSITION_MID = (POSITION_UP+POSITION_DOWN)/2;
  
  public double targetPosition=POSITION_UP;
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new IntakeArmCommand());
  }

  public void init(){
    if(getPosition() > POSITION_MID){
      setTargetPosition(getPosition());
    }
    else{
      setTargetPosition(POSITION_UP);
    }
  }

  public void setTargetPosition(double position){
    if(position < POSITION_MAX) {  // sensor values are negative 
      position=POSITION_MAX;
    }
    if(position > POSITION_DOWN) {  // sensor values are negative
      position=POSITION_DOWN;
    }
    if (position!=POSITION_DOWN) {
      RobotMap.intakeArmTalon.config_kP(0, 3.0, 0);
      RobotMap.intakeArmTalon.config_kD(0, 0.2, 0);
      RobotMap.intakeArmTalon.config_kI(0, 0.000, 0);
    } else {
      RobotMap.intakeArmTalon.config_kP(0, 0.5, 0);
    }
    targetPosition = position; 
    RobotMap.intakeArmTalon.set(ControlMode.Position, position);
  }

  public double getPosition(){
    return(RobotMap.intakeArmTalon.getSelectedSensorPosition());
  }

  public double getTargetPosition(){
    return(targetPosition);
  }

  
}
