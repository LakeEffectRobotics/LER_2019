/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.instant;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

/**
 * Add your docs here.
 */
public class BumpElevatorHeight extends InstantCommand {
  public static final boolean UP = true;
  public static final boolean DOWN = false;
  

  boolean direction;
  /**
   * Add your docs here.
   */
  public BumpElevatorHeight(boolean direction) {
    super();
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevator);
    
    this.direction = direction;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    double height = Robot.elevator.getHeight();
    
    if(height > Elevator.MAX_HEIGHT && direction == UP){
      return;
    }
    if(height < Elevator.GROUND_HEIGHT && direction == DOWN){
      return;
    }

    for(int i = 0; i<Elevator.HEIGHTS.length; i++){
      //If step is lower then current, and next step is higher
      if(height >= Elevator.HEIGHTS[i] && height <= Elevator.HEIGHTS[i+1]){
        if(direction == UP){
          //Go to higher step
          Robot.elevator.setTargetHeight(Elevator.HEIGHTS[i+1], 0);
        }else if(direction == DOWN){
          //Go to lower step
          Robot.elevator.setTargetHeight(Elevator.HEIGHTS[i], 0);
        }
      }

    }
  }

}
