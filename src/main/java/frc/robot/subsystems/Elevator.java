/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ElevatorCommand;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum Mode{
    CARGO, HATCH
  }

  //TODO: Set heights
  public static final double GROUND_HEIGHT = 37;
  public static final double MAX_HEIGHT = 80.75;
  
  public static final double LOW_HEIGHT = 50;
  public static final double MID_HEIGHT = 60;
  public static final double HIGH_HEIGHT = 70;

  public static final double[] HEIGHTS = {GROUND_HEIGHT, LOW_HEIGHT, MID_HEIGHT, HIGH_HEIGHT, MAX_HEIGHT};

  public static final double HATCH_OFFSET = -5;
  public static final double HATCH_RELEASE_OFFSET = -5;

  public static final double acceleration = 1;

  double targetHeight = GROUND_HEIGHT;
  public Mode currentMode = Mode.CARGO;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorCommand());
  }


  public double getHeight(){
    // return(RobotMap.elevatorSpark1.getEncoder().getPosition());
    return 0;
  }

  public double getTargetHeight(){
    return(targetHeight);
  }

  public void setTargetHeight(double target, double offset, String src){
    if(target > MAX_HEIGHT){
      target=MAX_HEIGHT;
    }
    if(target < GROUND_HEIGHT){
      System.out.println(target+"is less than"+GROUND_HEIGHT);
      target = GROUND_HEIGHT;
    }
    targetHeight = target;  

    // System.out.print("T0:"+src+"\t");

    // System.out.print("T1:"+targetHeight+"\t");

    //Add offset here so it doesn't affect the saved target
    target += offset;
    if(currentMode == Mode.HATCH) target += HATCH_OFFSET;
    if(target > MAX_HEIGHT) target=MAX_HEIGHT;
    if(target < GROUND_HEIGHT) target = GROUND_HEIGHT;

    // System.out.println("T2:"+target);
    RobotMap.elevatorSpark1.getPIDController().setReference(target, ControlType.kPosition);
  }

  public void setOffset(double offset){
    setTargetHeight(getTargetHeight(), offset, "Offset");
  }

  public Mode getMode(){
    return(currentMode);
  }

  public void setMode(Mode m){
    currentMode = m;
    //Update height to current mode
    setTargetHeight(targetHeight, 0, "Setmode");
  }
}
