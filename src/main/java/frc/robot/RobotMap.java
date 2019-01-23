/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the right and right motors, you could define the
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
	 * TODO: Update pins once Sparks are physically mounted
	 */
		final static int RIGHT_MAX_1 = 2;
		final static int RIGHT_MAX_2 = 3;
		final static int RIGHT_MAX_3 = 4;
		final static int LEFT_MAX_1 = 5;
		final static int LEFT_MAX_2 = 6;
		final static int LEFT_MAX_3 = 7;	

	/**
	 * Creating motor controller objects
	 */
		public static CANSparkMax rightDriveMax1 = new CANSparkMax(RIGHT_MAX_1, MotorType.kBrushless);
		public static CANSparkMax rightDriveMax2 = new CANSparkMax(RIGHT_MAX_2, MotorType.kBrushless);
		public static CANSparkMax rightDriveMax3 = new CANSparkMax(RIGHT_MAX_3, MotorType.kBrushless);
		public static CANSparkMax leftDriveMax1 = new CANSparkMax(LEFT_MAX_1, MotorType.kBrushless);
		public static CANSparkMax leftDriveMax2 = new CANSparkMax(LEFT_MAX_2, MotorType.kBrushless);
		public static CANSparkMax leftDriveMax3 = new CANSparkMax(LEFT_MAX_3, MotorType.kBrushless);
	/**
	 * Creating Gyro object
	 */	
	public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	public static void init() {
		//Set followers
		rightDriveMax2.follow(rightDriveMax1);
		rightDriveMax3.follow(rightDriveMax1);
		leftDriveMax2.follow(leftDriveMax1);
		leftDriveMax3.follow(leftDriveMax1);
	}
}
