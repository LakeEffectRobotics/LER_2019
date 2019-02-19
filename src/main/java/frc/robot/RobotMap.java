/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.IntakeArm;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Talon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
	 * Declaring CANIDs
	 */
		final static int RIGHT_SPARK_1 = 1;
		final static int RIGHT_SPARK_2 = 2;
		final static int RIGHT_SPARK_3 = 3;
		final static int LEFT_SPARK_1 = 4;
		final static int LEFT_SPARK_2 = 5;
		final static int LEFT_SPARK_3 = 6;	


		final static int INTAKE_ARM_TALON = 7;
		final static int INTAKE_ARM_VICTOR = 8;
		final static int INTAKE_ROLLER_VICTOR = 9;
	/**
	 * Creating motor controller objects
	 */
		public static CANSparkMax rightDriveSpark1 = new CANSparkMax(RIGHT_SPARK_1, MotorType.kBrushless);
		public static CANSparkMax rightDriveSpark2 = new CANSparkMax(RIGHT_SPARK_2, MotorType.kBrushless);
		public static CANSparkMax rightDriveSpark3 = new CANSparkMax(RIGHT_SPARK_3, MotorType.kBrushless);
		public static CANSparkMax leftDriveSpark1 = new CANSparkMax(LEFT_SPARK_1, MotorType.kBrushless);
		public static CANSparkMax leftDriveSpark2 = new CANSparkMax(LEFT_SPARK_2, MotorType.kBrushless);
		public static CANSparkMax leftDriveSpark3 = new CANSparkMax(LEFT_SPARK_3, MotorType.kBrushless);

		public static TalonSRX intakeArmTalon = new TalonSRX(INTAKE_ARM_TALON);
		public static VictorSPX intakeArmVictor = new VictorSPX(INTAKE_ARM_VICTOR);

		public static VictorSPX intakeRollerVictor = new VictorSPX(INTAKE_ROLLER_VICTOR);
	/**
	 * Creating Gyro object
	 */	
	public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	public static void init() {
		//Set followers
		rightDriveSpark2.follow(rightDriveSpark1);
		rightDriveSpark3.follow(rightDriveSpark1);
		leftDriveSpark2.follow(leftDriveSpark1);
		leftDriveSpark3.follow(leftDriveSpark1);

		intakeArmVictor.follow(intakeArmTalon);
	}
}
