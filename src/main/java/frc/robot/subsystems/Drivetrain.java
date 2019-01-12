/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commands.DefaultTeleopDriveCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;


public class Drivetrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private final double NATIVE_TO_INCHES = 212.77777777777777777777777777778; // TODO Make this an actually real value
	public final double MAX_INCHES_PER_SECOND = 20;
	private double l_prev_value = 0;
	private double r_prev_value = 0;
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new DefaultTeleopDriveCommand());
	}
	
	public void setPosition(double l, double r) {
		RobotMap.left_drive_spark_1.setPosition(l * NATIVE_TO_INCHES);
		RobotMap.left_drive_spark_2.setPosition(l * NATIVE_TO_INCHES);
		RobotMap.left_drive_spark_3.setPosition(l * NATIVE_TO_INCHES);
		RobotMap.right_drive_spark_1.setPosition(-r * NATIVE_TO_INCHES);
		RobotMap.right_drive_spark_2.setPosition(-r * NATIVE_TO_INCHES);
		RobotMap.right_drive_spark_3.setPosition(-r * NATIVE_TO_INCHES);
	}
	
	public void setSpeed(double l, double r) {
		RobotMap.left_drive_spark_1.setPosition(l);
		RobotMap.left_drive_spark_2.setPosition(l);
		RobotMap.left_drive_spark_3.setPosition(l);
		RobotMap.right_drive_spark_1.setPosition(-r);
		RobotMap.right_drive_spark_2.setPosition(-r);
		RobotMap.right_drive_spark_3.setPosition(-r);
	}
	
	/**
	 * Spark appears to not have any sort of sensors
	 * 
	public double getLeftEncoderPosition() {
		return (-RobotMap.left_drive_spark_1.getSelectedSensorPosition(0) / NATIVE_TO_INCHES) - l_prev_value; // TODO May need to make negative
	}
	
	public double getRightEncoderPosition() {
		return (RobotMap.right_drive_talon_1.getSelectedSensorPosition(0) / NATIVE_TO_INCHES) - r_prev_value; // TODO May need to make negative
	}
	
	public void resetEncoderPosition() {
		r_prev_value = RobotMap.right_drive_talon_1.getSelectedSensorPosition(0) / NATIVE_TO_INCHES;
		l_prev_value = -RobotMap.left_drive_talon_1.getSelectedSensorPosition(0) / NATIVE_TO_INCHES;
	}

	public Object[] getAutoDrive(double speed, double distance, boolean line_stop) {
		return getAutoDrive(speed, distance, line_stop, 0);
	}
	
	public Object[] getAutoDrive(double speed, double distance, boolean line_stop, double slow_down) {
		final double SLOW_DOWN_THRESHOLD = slow_down;
    	boolean finished = false;
    	double l = 0;
    	double r = 0;
    	double average_encoder_position = (getLeftEncoderPosition() + getRightEncoderPosition()) / 2;
    	double distance_left = distance - average_encoder_position;
    	
    	if (distance > 0 && distance_left > SLOW_DOWN_THRESHOLD) {
    		l = speed;
    	}
    	else if (distance < 0 && distance_left < -SLOW_DOWN_THRESHOLD) {
    		l = -speed;
    	}
    	
    	else if (distance > 0 && distance_left > 0) {
    		l = speed * (Math.abs(distance_left) / SLOW_DOWN_THRESHOLD);
    	}
    	else if (distance < 0 && distance_left < 0){
    		l = -speed * (Math.abs(distance_left) / SLOW_DOWN_THRESHOLD);
		}
		else {
    		finished = true;
    	}
    	
    	r = l;
    	
    	return new Object[] {l, r, finished};
	}
	 */
}
