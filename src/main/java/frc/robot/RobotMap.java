/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

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
	 * TODO: Update pin mapping
	 */
	private static final int LEFT_TALON_1 = 1;
	private static final int LEFT_TALON_2 = 2;
	private static final int LEFT_TALON_3 = 3;
	private static final int RIGHT_TALON_1 = 12;
	private static final int RIGHT_TALON_2 = 11;//Victor SPX
	private static final int RIGHT_TALON_3 = 10;
	

	/**
	 * Creating motor controller objects
	 */
	public static TalonSRX left_drive_talon_1 = new TalonSRX(LEFT_TALON_1);
	public static TalonSRX left_drive_talon_2 = new TalonSRX(LEFT_TALON_2);
	public static TalonSRX left_drive_talon_3 = new TalonSRX(LEFT_TALON_3);
	public static TalonSRX right_drive_talon_1 = new TalonSRX(RIGHT_TALON_1);
	public static VictorSPX right_drive_talon_2 = new VictorSPX(RIGHT_TALON_2);
	public static TalonSRX right_drive_talon_3 = new TalonSRX(RIGHT_TALON_3);
	
	/**
	 * Creating Gyro object
	 */	
	public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	public static void init() {
		
		left_drive_talon_2.follow(left_drive_talon_1);
		left_drive_talon_3.follow(left_drive_talon_1);
		right_drive_talon_2.follow(right_drive_talon_1);
		right_drive_talon_3.follow(right_drive_talon_1);	

		left_drive_talon_1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,PID_MODE,0);
		right_drive_talon_1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,PID_MODE,0);
	}
}
