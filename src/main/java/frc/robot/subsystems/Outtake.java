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
import frc.robot.Tools;
import frc.robot.commands.OuttakeCommand;

/**
 * Add your docs here.
 */
public class Outtake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /*
   * public static double L_GRIP = 0.6; Original outtake public static double L_IN
   * = 0.51; public static double L_OUT = 0; public static double R_GRIP = 0.5;
   * public static double R_IN = 0.41; public static double R_OUT = 1;
   */

  public static final int L_GRIP = 0;
  public static final int L_IN = 0;
  public static final int L_OUT = 20;
  public static final int R_GRIP = 0;
  public static final int R_IN = 0;
  public static final int R_OUT = 20;

  public static final int SIDE_RIGHT = 1;
  public static final int SIDE_LEFT = 0;
  public static final int SIDE_NONE = -1;

  final static double P = 0.1;

  /***The maxium distance from the line, in inches */
  public static final int MAX_DIST = 4;

  int lTarget = 0;
  int lPos = 0;

  int rTarget = 0;
  int rPos = 0;

  public int lastSide = SIDE_NONE;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new OuttakeCommand());
  }

  public void setSide(int side, double t) {
    int target = (int) t;

    if(target == 0){
      target = -1;
    }

    if (side == SIDE_LEFT) {
      lTarget = target;
    }

    if (side == SIDE_RIGHT) {
      rTarget = target;
    }

    System.out.println(side);
  }

  public void drive() {

    int lDelta = lPos - lTarget;
    double lSpeed = lDelta * -P;

    if(lTarget < 0 && lDelta < 2){
      lTarget --;
    }
    
    int rDelta = rPos - rTarget;
    double rSpeed = rDelta * -P;

    
    if(rTarget < 0 && rDelta < 2){
      rTarget --;
    }

    RobotMap.leftOuttakeVictor.set(ControlMode.PercentOutput, Tools.fitToRange(-lSpeed, -1, 1));
    RobotMap.rightOuttakeVictor.set(ControlMode.PercentOutput, Tools.fitToRange(rSpeed, -1, 1));

    if(lSpeed > 0){
			lPos += RobotMap.leftOuttakeCounter.get();
		}
		if(lSpeed < 0){
			lPos -= RobotMap.leftOuttakeCounter.get();
		}
    // System.out.println("L: "+lDelta+"\t"+lPos+"\t"+RobotMap.leftOuttakeCounter.get());
    RobotMap.leftOuttakeCounter.reset();
    
    if(rSpeed > 0){
      rPos += RobotMap.rightOuttakeCounter.get();
		}
		if(rSpeed < 0){
			rPos -= RobotMap.rightOuttakeCounter.get();
    }
    // System.out.println("R: "+rDelta+"\t"+rPos+"\t"+RobotMap.rightOuttakeCounter.get());    
    RobotMap.rightOuttakeCounter.reset();

    if(RobotMap.leftOuttakeLimit.get()){
      lPos = 0;
      lTarget = 0;
    }
    if(RobotMap.rightOuttakeLimit.get()){
      rPos = 0;
      rTarget = 0;
    }

  }

}
