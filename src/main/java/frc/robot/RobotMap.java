/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	public static final int PID_MODE = 0;  //0 for closed loop 1 for cascaded closed loop

	/**
	 * Declaring pins
	 * TODO: Update pin once Sparks are physically mounted
	 */
	private static final int LEFT_SPARK_1 = 1;
	private static final int LEFT_SPARK_2 = 2;
	private static final int LEFT_SPARK_3 = 3;
	private static final int RIGHT_SPARK_1 = 4;
	private static final int RIGHT_SPARK_2 = 5;
	private static final int RIGHT_SPARK_3 = 6;
	

	/**
	 * Creating motor controller objects
	 */
	public static Spark left_drive_spark_1 = new Spark(LEFT_SPARK_1);
	public static Spark left_drive_spark_2 = new Spark(LEFT_SPARK_2);
	public static Spark left_drive_spark_3 = new Spark(LEFT_SPARK_3);
	public static Spark right_drive_spark_1 = new Spark(RIGHT_SPARK_1);
	public static Spark right_drive_spark_2 = new Spark(RIGHT_SPARK_2);
	public static Spark right_drive_spark_3 = new Spark(RIGHT_SPARK_3);
	
	/**
	 * Creating Gyro object
	 */	
	public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	public static void init() {
		////
	}
}
