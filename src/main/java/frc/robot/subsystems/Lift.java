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
  //  TODO: Update these values
  public final static int MIN_POSITION = 2708;
  public final static int MAX_POSITION = 2708;

  public final static int ROCKET_PORT_HEIGHT_1 = 2708;
  public final static int ROCKET_PORT_HEIGHT_2 = 2708;
  public final static int ROCKET_PORT_HEIGHT_3 = 2708;
  public final static int SHIP_PORT_HEIGHT = 2708;

  public final static int ROCKET_HATCH_HEIGHT_1 = 2708;
  public final static int ROCKET_HATCH_HEIGHT_2 = 2708;
  public final static int ROCKET_HATCH_HEIGHT_3 = 2708;
  public final static int SHIP_HATCH_HEIGHT = ROCKET_HATCH_HEIGHT_1;

  public final static int GROUND_HEIGHT = 2708;
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new LiftCommand());
  }
  
}
