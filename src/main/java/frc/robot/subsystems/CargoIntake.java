/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commands.CargoIntakeCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class CargoIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  //  TODO: Experimentally determine the actual value of the following variables
  final static int UP_POSITION = 2708;
  final static int DOWN_POSITION = 2708;

  public static enum Position {UP, DOWN};

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new CargoIntakeCommand());
  }

  public void setSpeed(double speed) {
    RobotMap.rollerVictor.set(ControlMode.PercentOutput, speed);
    RobotMap.conveyerVictor.set(ControlMode.PercentOutput, speed);
  }

  //  TODO: Put this method in OI.java
  //  TODO: Make sure that the cargo intake mechanism remains in place; if it slides down, then write the position code similar to that of the lift
  public void setPosition(Position position) {
    switch (position) {
      case UP:
        RobotMap.cargoIntakeTalon.setSelectedSensorPosition(UP_POSITION);
      case DOWN:
        RobotMap.cargoIntakeTalon.setSelectedSensorPosition(DOWN_POSITION);
    }
  }
}
