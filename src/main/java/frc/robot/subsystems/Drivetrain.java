/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCommand;
import edu.wpi.first.wpilibj.command.Subsystem;


public class Drivetrain extends Subsystem {
	public boolean shawnDriveIsActive = false;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DriveCommand());
	}
	
	public void drive(double l, double r) {
		//Other motors are followers
		
		if (!shawnDriveIsActive) {
			//  Standard drive
			RobotMap.leftDriveSpark1.set(l);
			RobotMap.rightDriveSpark1.set(-r);  //  r is inverted because the left and right motors are oriented in opposite directions
		} else {
			//  Drive with robot's front/back switched
			RobotMap.leftDriveSpark1.set(-r);
			RobotMap.rightDriveSpark1.set(l);
		}
	}

	public double[] getInitialEncoderPositions() {
		double l = RobotMap.leftDriveSpark1.getEncoder().getPosition();
		double r = -RobotMap.rightDriveSpark1.getEncoder().getPosition();
		return new double[] {l, r};
	}

	public double getAverageEncoderPosition(double[] initialEncoderPositions) {
		double l = RobotMap.leftDriveSpark1.getEncoder().getPosition() - initialEncoderPositions[0];
		double r = -RobotMap.rightDriveSpark1.getEncoder().getPosition() - initialEncoderPositions[1];
		return (l + r) / 2;
	}

	public Object[] getAutoDriveOutput(double speed, double distance, double[] initialEncoderPositions, double time, boolean line_stop) {
		//	TODO: Update these values
		final double SLOW_DOWN_THRESHOLD = 15.75;	//	Originally 0.4 m or 40 cm, now 15.75"
    	final double MIN_SPEED = 0.2;
		final double FINISHED_TOLERANCE = 2;		//	Originally 0.05 m or 5 cm, now 2"
		
    	boolean finished = false;
    	double l = 0;
    	double r = 0;
		/*
		double averageEncoderPosition = time;	//	Unsure as to why time is being used in place of the encoders; perhaps because it's a good approximation? Or were the encoders having issues?
		*/
		double averageEncoderPosition = getAverageEncoderPosition(initialEncoderPositions);
		double distanceLeft = distance - averageEncoderPosition;
    	if (distanceLeft > SLOW_DOWN_THRESHOLD) {
    		l = speed;
    	} else if (distanceLeft < -SLOW_DOWN_THRESHOLD) {
    		l = -speed;
		} else if (distanceLeft > 0) {
    		l = speed * (distanceLeft / SLOW_DOWN_THRESHOLD);
    	} else {
			l = -speed * (distanceLeft / SLOW_DOWN_THRESHOLD);
		}
		
    	if (l < MIN_SPEED && l > 0) {
    		l = MIN_SPEED;
    	} else if (l < -MIN_SPEED && l < 0) {
    		l = -MIN_SPEED;
		}
		
    	// if (Math.abs(distanceLeft) < FINISHED_TOLERANCE || ((RobotMap.leftTapeSensor2.get() || RobotMap.rightTapeSensor2.get()) && line_stop)) {
		// 	finished = true;
    	// }
    	
    	r = l;
    	
    	return new Object[] {l, r, finished};
	}	
}
