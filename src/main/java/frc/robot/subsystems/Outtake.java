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
  /* Commented out by Mr. Wood - these are values for the twin before replacing pots???
  public static final int L_GRIP = 200;
  public static final int L_IN = 226;
  public static final int L_OUT = 500;
  public static final int R_GRIP = -220;
  public static final int R_IN = -248;
  public static final int R_OUT = -410;*/

  public static final int L_GRIP = 3410;
  public static final int L_IN = 3382;//-555;
  public static final int L_OUT = 3148;
  public static final int R_GRIP = -234;
  public static final int R_IN = -254;
  public static final int R_OUT = -500;

  public static final boolean L_DISABLED = false;
  public static final boolean R_DISABLED = false;

  public static final int SIDE_RIGHT = 1;
  public static final int SIDE_LEFT = 0;
  public static final int SIDE_NONE = -1;

  /***The maxium distance from the line, in inches */
  public static final int MAX_DIST = 4;

  int lTarget = -1;

  int rTarget = -1;

  public int lastSide = SIDE_NONE;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new OuttakeCommand());
  }

  public void setSide(int side, int t) {
    int target = (int) t;
    if(t == R_GRIP || t == L_GRIP){
      System.out.println("GRIP 2");
    }

    if (side == SIDE_LEFT) {
      lTarget = target;
      // if(Robot.outtake.L_DISABLED){
      //   return;
      // }
    }

    if (side == SIDE_RIGHT) {
      rTarget = target;
    }
    RobotMap.leftOuttakeTalon.set(ControlMode.Position,lTarget);
    RobotMap.rightOuttakeTalon.set(ControlMode.Position,rTarget);
    //System.out.println(side);
  }

  public void drive() {

    /*int lDelta = lPos - lTarget;
    double lSpeed = lDelta * P;


    if(lTarget < 0 && lDelta < 2){
      lTarget --; //Shift target to keep moving
    }
    
    int rDelta = rPos - rTarget;
    double rSpeed = rDelta * -P;*/

    /*if (lTarget>0) {
      lRecentReset=false;
    }
    if (rTarget>0) {
      rRecentReset=false;
    }

    if (!RobotMap.leftOuttakeLimit.get()) {
      lLimitCount++;
    } else {
      lLimitCount=0;
    }
    if (!RobotMap.rightOuttakeLimit.get()) {
      rLimitCount++;
    } else {
      rLimitCount=0;
    }

    if (lLimitCount>5 && !lRecentReset) {
      lPos=0;
      if (lTarget==-1) lTarget=0;
      lRecentReset=true;
    }
    if (rLimitCount>8 && !rRecentReset) {
      rPos=0;
      if (rTarget==-1) rTarget=0;
      rRecentReset=true;
    }

    if (RobotMap.leftOuttakeVictor.getMotorOutputVoltage()>0) {
			lPos += RobotMap.leftOuttakeCounter.get();
		} else {
      lPos -= RobotMap.leftOuttakeCounter.get();
    }
    RobotMap.leftOuttakeCounter.reset();

    if (RobotMap.rightOuttakeVictor.getMotorOutputVoltage()>0) {
      rPos += RobotMap.rightOuttakeCounter.get();
    } else {
      rPos -= RobotMap.rightOuttakeCounter.get();
    }
    RobotMap.rightOuttakeCounter.reset();

    if (lPos<-1) {
      lPos=lTarget-1;
    }
    if (rPos<-1) {
      rPos=rTarget-1;
    }


    int lDelta = lPos - lTarget;
    double lSpeed = lDelta * P;

    int rDelta = rPos - rTarget;
    double rSpeed = rDelta * P;

    //RobotMap.leftOuttakeVictor.set(ControlMode.PercentOutput, Tools.fitToRange(-lSpeed, -0.75, 0.75));
    //RobotMap.rightOuttakeVictor.set(ControlMode.PercentOutput, Tools.fitToRange(-rSpeed, -0.75, 0.75));
    if (lDelta<-5) {
      RobotMap.leftOuttakeVictor.set(ControlMode.PercentOutput,.75);
    } else if (lDelta<0) {
      RobotMap.leftOuttakeVictor.set(ControlMode.PercentOutput,.15);
    } else if (lDelta>5) {
      RobotMap.leftOuttakeVictor.set(ControlMode.PercentOutput,-.75);
    } else {
      RobotMap.leftOuttakeVictor.set(ControlMode.PercentOutput,-.15);
    }

    if (rDelta<-5) {
      RobotMap.rightOuttakeVictor.set(ControlMode.PercentOutput,.75);
    } else if (rDelta<0) {
      RobotMap.rightOuttakeVictor.set(ControlMode.PercentOutput,.15);
    } else if (rDelta>5) {
      RobotMap.rightOuttakeVictor.set(ControlMode.PercentOutput,-.75);
    } else {
      RobotMap.rightOuttakeVictor.set(ControlMode.PercentOutput,-.15);
    }
    
    /*if(rTarget < 0 && rDelta < 2){
      rTarget --; //Shift target to keep moving
    }

    //if(lSpeed > 0){
    if (RobotMap.leftOuttakeVictor.getMotorOutputVoltage()>0) {
			lPos += RobotMap.leftOuttakeCounter.get();
		} else {
		//if(lSpeed < 0){
      lPos -= RobotMap.leftOuttakeCounter.get();
    }
    //if(rSpeed > 0){
    if (RobotMap.rightOuttakeVictor.getMotorOutputVoltage()>0) {
      rPos += RobotMap.rightOuttakeCounter.get();
    } else {
    //if(rSpeed < 0){
      rPos -= RobotMap.rightOuttakeCounter.get();
    }
 
    RobotMap.leftOuttakeVictor.set(ControlMode.PercentOutput, Tools.fitToRange(-lSpeed, -0.75, 0.75));
    RobotMap.rightOuttakeVictor.set(ControlMode.PercentOutput, Tools.fitToRange(rSpeed, -0.75, 0.75));

   System.out.println("L: "+lDelta+"\t"+lPos+"\t"+!RobotMap.leftOuttakeLimit.get()+"\t"+lTarget);
    RobotMap.leftOuttakeCounter.reset();


    // System.out.println("R: "+rDelta+"\t"+rPos+"\t"+RobotMap.rightOuttakeCounter.get());    
    RobotMap.rightOuttakeCounter.reset();

    System.out.println(lSpeed+"\t"+lDelta+"\t"+!RobotMap.leftOuttakeLimit.get());

    if(!RobotMap.leftOuttakeLimit.get()){
      lPos = 0;
      lTarget = 4; // Move back off the limit switch
    }
    if(!RobotMap.rightOuttakeLimit.get()){
      rPos = 0;
      rTarget = 4; // Move back off the limit switch
    }
*/
  }
}
