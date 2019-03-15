/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.Colour;

public class LightCommand extends Command {
  public LightCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.lights);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(RobotMap.outerLeftSensor.get()){
      Robot.lights.setColour(Lights.LEFT, Colour.BLUE);
      if(RobotMap.innerLeftSensor.get()) 
        Robot.lights.setColour(Lights.LEFT, Colour.GREEN);
    }   
    if(RobotMap.outerRightSensor.get()){
      Robot.lights.setColour(Lights.RIGHT, Colour.BLUE);
      if(RobotMap.innerRightSensor.get()) 
        Robot.lights.setColour(Lights.RIGHT, Colour.GREEN);
    }    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
