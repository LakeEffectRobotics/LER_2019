/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.LiftCommand;
import frc.robot.RobotMap;
import frc.robot.Tools;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  //  TODO: Experimentally determine the following constant values
  final static double MIN_POSITION = 2708;
  final static double MAX_POSITION = 2708;

  final static double ROCKET_PORT_HEIGHT_1 = 2708;
  final static double ROCKET_PORT_HEIGHT_2 = 2708;
  final static double ROCKET_PORT_HEIGHT_3 = 2708;
  final static double SHIP_PORT_HEIGHT = 2708;

  final static double ROCKET_HATCH_HEIGHT_1 = 2708;
  final static double ROCKET_HATCH_HEIGHT_2 = 2708;
  final static double ROCKET_HATCH_HEIGHT_3 = 2708;
  final static double SHIP_HATCH_HEIGHT = ROCKET_HATCH_HEIGHT_1;

  final static double GROUND_HEIGHT = 2708;
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new LiftCommand());
  }
  
  public double getPosition() {
    return RobotMap.liftTalon.getSelectedSensorPosition(0);
  }
  
  public double getTargetPosition() {
    return RobotMap.liftTalon.getActiveTrajectoryPosition();
  }
  
  public void setTargetPosition(double position) {
    position = Tools.fitToRange(position, MIN_POSITION, MAX_POSITION);
    RobotMap.liftTalon.setSelectedSensorPosition((int) position, 0, 0);
  }
}
