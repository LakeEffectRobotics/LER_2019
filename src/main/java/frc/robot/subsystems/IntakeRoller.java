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
import frc.robot.commands.IntakeRollerCommand;

/**
 * Add your docs here.
 */
public class IntakeRoller extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  final double SPEED_PRACTICE = 0.7;
  final double SPEED = 1;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new IntakeRollerCommand());
  }

  public void spin(double speed){
    //TODO change for comp bot
    RobotMap.intakeRollerTalon.set(ControlMode.PercentOutput, -speed*SPEED_PRACTICE);
  }

}
