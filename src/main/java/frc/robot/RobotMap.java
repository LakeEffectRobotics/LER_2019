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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.components.TalonSRX_2;
import frc.robot.components.TapeSensor;
import frc.robot.subsystems.Climber;

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
	final static int RIGHT_DRIVE_SPARK_1 = 1;
	final static int RIGHT_DRIVE_SPARK_2 = 2;
	final static int RIGHT_DRIVE_SPARK_3 = 3;
	final static int LEFT_DRIVE_SPARK_1 = 4;
	final static int LEFT_DRIVE_SPARK_2 = 5;
	final static int LEFT_DRIVE_SPARK_3 = 6;

	final static int ROLLER_VICTOR = 2708;
	final static int CARGO_INTAKE_TALON = 2708;
	final static int CARGO_INTAKE_VICTOR = 2708;
	final static int CONVEYOR_VICTOR = 2708;
	final static int LIFT_TALON = 2708;
	final static int CLIMBER_TALON_1 = 2708;
	final static int CLIMBER_TALON_2 = 2708;

	//	Sensor 1 is in the front, Sensor 2 is in the center, Sensor 3 is in the back
	final static int LEFT_TAPE_SENSOR_1 = 0;
	final static int LEFT_TAPE_SENSOR_2 = 1;
	//	final static int LEFT_TAPE_SENSOR_3 = 2708;

	//	final static int CENTER_TAPE_SENSOR_1 = 2708;
	//	final static int CENTER_TAPE_SENSOR_2 = 2708;
	//	final static int CENTER_TAPE_SENSOR_3 = 2708;

	final static int RIGHT_TAPE_SENSOR_1 = 2;
	final static int RIGHT_TAPE_SENSOR_2 = 3;
	//	final static int RIGHT_TAPE_SENSOR_3 = 2708;

	final static int INTAKE_ARM_TALON = 7;
	final static int INTAKE_ARM_VICTOR = 8;
	final static int INTAKE_ROLLER_TALON = 9;

	final static int ELEVATOR_TALON = 10;
	final static int ELEVATOR_VICTOR = 11;
	/**
	 * Creating motor controller objects
	 */
	public static CANSparkMax rightDriveSpark1 = new CANSparkMax(RIGHT_DRIVE_SPARK_1, MotorType.kBrushless);
	public static CANSparkMax rightDriveSpark2 = new CANSparkMax(RIGHT_DRIVE_SPARK_2, MotorType.kBrushless);
	public static CANSparkMax rightDriveSpark3 = new CANSparkMax(RIGHT_DRIVE_SPARK_3, MotorType.kBrushless);
	public static CANSparkMax leftDriveSpark1 = new CANSparkMax(LEFT_DRIVE_SPARK_1, MotorType.kBrushless);
	public static CANSparkMax leftDriveSpark2 = new CANSparkMax(LEFT_DRIVE_SPARK_2, MotorType.kBrushless);
	public static CANSparkMax leftDriveSpark3 = new CANSparkMax(LEFT_DRIVE_SPARK_3, MotorType.kBrushless);

	public static TalonSRX intakeArmTalon = new TalonSRX(INTAKE_ARM_TALON);
	public static VictorSPX intakeArmVictor = new VictorSPX(INTAKE_ARM_VICTOR);
	public static TalonSRX intakeRollerTalon = new TalonSRX(INTAKE_ROLLER_TALON);

	public static TalonSRX elevatorTalon = new TalonSRX(ELEVATOR_TALON);
	public static VictorSPX elevatorVictor = new VictorSPX(ELEVATOR_VICTOR);

	public static VictorSPX conveyerVictor = new VictorSPX(CONVEYOR_VICTOR);
	public static TalonSRX_2 climberTalon1 = new TalonSRX_2(CLIMBER_TALON_1, Climber.DOWN_POSITION,
			Climber.UP_POSITION);
	public static TalonSRX_2 climberTalon2 = new TalonSRX_2(CLIMBER_TALON_2, Climber.DOWN_POSITION, Climber.UP_POSITION);
	/**
	 * Creating Gyro object
	 */	
	public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	/**
	 * Creating sensor objects
	 */
	public static TapeSensor leftTapeSensor1 = new TapeSensor(LEFT_TAPE_SENSOR_1);
	public static TapeSensor leftTapeSensor2 = new TapeSensor(LEFT_TAPE_SENSOR_2);
	//	public static TapeSensor leftTapeSensor3 = new TapeSensor(LEFT_TAPE_SENSOR_3);

	//	public static TapeSensor centerTapeSensor1 = new TapeSensor(CENTER_TAPE_SENSOR_1);
	//	public static TapeSensor centerTapeSensor2 = new TapeSensor(CENTER_TAPE_SENSOR_2);
	//	public static TapeSensor centerTapeSensor3 = new TapeSensor(CENTER_TAPE_SENSOR_3);

	public static TapeSensor rightTapeSensor1 = new TapeSensor(RIGHT_TAPE_SENSOR_1);
	public static TapeSensor rightTapeSensor2 = new TapeSensor(RIGHT_TAPE_SENSOR_2);
	//	public static TapeSensor rightTapeSensor3 = new TapeSensor(RIGHT_TAPE_SENSOR_3);
	
	public static void init() {
		//Set followers
		rightDriveSpark2.follow(rightDriveSpark1);
		rightDriveSpark3.follow(rightDriveSpark1);
		leftDriveSpark2.follow(leftDriveSpark1);
		leftDriveSpark3.follow(leftDriveSpark1);

		intakeArmVictor.follow(intakeArmTalon);
		
		elevatorVictor.follow(elevatorTalon);
		elevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PID_MODE, 0);
		elevatorTalon.config_kP(0, 2, 0);
		climberTalon2.follow(climberTalon1);
	}
}
