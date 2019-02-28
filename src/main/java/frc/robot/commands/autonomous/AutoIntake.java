/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.commands.instant.ToggleIntakeArm;

public class AutoIntake extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoIntake() {
    //Spin rollers until a cargo is grabbed
    addSequential(new AutoSpinRoller(0.75));
    addSequential(new WaitForSensor(RobotMap.intakeLimitSwitch, true));
    
    //Stop rollers and raise arm
    addSequential(new AutoSpinRoller(0));
    addSequential(new ToggleIntakeArm());
    addSequential(new WaitCommand(0.2)); // Give time to raise, can be tweaked for optimal performance

    //Outtake ball to flippers
    addSequential(new AutoSpinRoller(0.5));
    addSequential(new WaitForSensor(RobotMap.intakeLimitSwitch, false));

    //Lower intake
    addSequential(new ToggleIntakeArm());
    addSequential(new WaitCommand(0.1)); // Give time to lower, can be tweaked for optimal performance
    addSequential(new AutoSpinRoller(0.5));

  }
}
