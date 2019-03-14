/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.components.TalonSRX_2;
import frc.robot.components.TapeSensor;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;

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

	// final static int CLIMBER_TALON_1 = 2708;
	// final static int CLIMBER_TALON_2 = 2708;

	//	Sensor 1 is in the front, Sensor 2 is in the center, Sensor 3 is in the back
	final static int LEFT_TAPE_SENSOR_1 = 2;
	// final static int LEFT_TAPE_SENSOR_2 = 0;
	//	final static int LEFT_TAPE_SENSOR_3 = 2708;

	//	final static int CENTER_TAPE_SENSOR_1 = 2708;
	//	final static int CENTER_TAPE_SENSOR_2 = 2708;
	//	final static int CENTER_TAPE_SENSOR_3 = 2708;

	final static int RIGHT_TAPE_SENSOR_1 = 1;
	// final static int RIGHT_TAPE_SENSOR_2 = 1;
	//	final static int RIGHT_TAPE_SENSOR_3 = 2708;

	final static int INTAKE_ARM_TALON = 10;
	final static int INTAKE_ARM_VICTOR = 20;
	final static int INTAKE_ROLLER_TALON = 12;
	final static int INTAKE_LIMIT_SWITCH = 3;
	//final static int INTAKE_POT = 2;

	final static int ELEVATOR_SPARK_1 = 7;
	final static int ELEVATOR_SPARK_2 = 8;

	final static int LEFT_SERVO = 1;
	final static int RIGHT_SERVO = 2;

	//LED pins
	//TODO set proper pins
	private static final int LEFT_PB_RELAY = 0;
	private static final int LEFT_GR_RELAY = 1;
	private static final int RIGHT_PB_RELAY = 2;
	private static final int RIGHT_GR_RELAY = 3;

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

	public static CANSparkMax elevatorSpark1 = new CANSparkMax(ELEVATOR_SPARK_1, MotorType.kBrushless);
	public static CANSparkMax elevatorSpark2 = new CANSparkMax(ELEVATOR_SPARK_2, MotorType.kBrushless);

	// public static TalonSRX_2 climberTalon1 = new TalonSRX_2(CLIMBER_TALON_1, Climber.DEPLOYED_POSITION,
	// 		Climber.DEFAULT_POSITION);
	// public static TalonSRX_2 climberTalon2 = new TalonSRX_2(CLIMBER_TALON_2, Climber.DEPLOYED_POSITION, Climber.DEFAULT_POSITION);

	public static Servo leftServo = new Servo(LEFT_SERVO);
	public static Servo rightServo = new Servo(RIGHT_SERVO);
	/**
	 * Creating Gyro object
	 */	
	public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	public static final DigitalInput intakeLimitSwitch = new DigitalInput(INTAKE_LIMIT_SWITCH);
	public static SerialPort jevoisSerial;

	
	//LED lights
	public static Relay leftLED_PB = new Relay(LEFT_PB_RELAY, Relay.Direction.kBoth);
	public static Relay leftLED_GB = new Relay(LEFT_GR_RELAY, Relay.Direction.kBoth);
	public static Relay rightLED_PB = new Relay(RIGHT_PB_RELAY, Relay.Direction.kBoth);
	public static Relay rightLED_GR = new Relay(RIGHT_GR_RELAY, Relay.Direction.kBoth);

	/**
	 * Creating sensor objects
	 */
	public static TapeSensor leftTapeSensor1 = new TapeSensor(LEFT_TAPE_SENSOR_1);
	// public static TapeSensor leftTapeSensor2 = new TapeSensor(LEFT_TAPE_SENSOR_2);
	//	public static TapeSensor leftTapeSensor3 = new TapeSensor(LEFT_TAPE_SENSOR_3);

	//	public static TapeSensor centerTapeSensor1 = new TapeSensor(CENTER_TAPE_SENSOR_1);
	//	public static TapeSensor centerTapeSensor2 = new TapeSensor(CENTER_TAPE_SENSOR_2);
	//	public static TapeSensor centerTapeSensor3 = new TapeSensor(CENTER_TAPE_SENSOR_3);

	public static TapeSensor rightTapeSensor1 = new TapeSensor(RIGHT_TAPE_SENSOR_1);
	// public static TapeSensor rightTapeSensor2 = new TapeSensor(RIGHT_TAPE_SENSOR_2);
	//	public static TapeSensor rightTapeSensor3 = new TapeSensor(RIGHT_TAPE_SENSOR_3);
	
	public static void init() { //r/outoftheloop
		//Set followers
		rightDriveSpark2.follow(rightDriveSpark1);
		rightDriveSpark3.follow(rightDriveSpark1);
		leftDriveSpark2.follow(leftDriveSpark1);
		leftDriveSpark3.follow(leftDriveSpark1);

		leftDriveSpark1.setOpenLoopRampRate(0.5);
		rightDriveSpark1.setOpenLoopRampRate(0.5);  

		intakeArmTalon.configOpenloopRamp(0.1);
		intakeArmTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		intakeArmTalon.setSensorPhase(true);
		intakeArmTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen);
		intakeArmTalon.config_kP(0, 1.0, 0);
		intakeArmTalon.config_kI(0, 0.0, 0);
		intakeArmTalon.config_kD(0, 0.001, 0);
		intakeArmVictor.follow(intakeArmTalon);

		elevatorSpark2.follow(elevatorSpark1, true);

		elevatorSpark1.getPIDController().setP(1);
		//Down is halved to prevent damage (somewhat)
		elevatorSpark1.getPIDController().setOutputRange(-Elevator.acceleration/2, Elevator.acceleration);
		elevatorSpark1.getPIDController().setReference(Elevator.GROUND_HEIGHT, ControlType.kPosition);
		elevatorSpark1.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		elevatorSpark1.getPIDController().setSmartMotionMaxVelocity(2000, 0);
		elevatorSpark1.getPIDController().setSmartMotionMaxAccel(1000, 0);

		Robot.elevator.setTargetHeight(Elevator.GROUND_HEIGHT, 0, "Init");

		// climberTalon2.follow(climberTalon1);

		//LEDs off by default:
		leftLED_PB.set(Relay.Value.kOff);
		leftLED_GB.set(Relay.Value.kOff);
		rightLED_PB.set(Relay.Value.kOff);
		rightLED_GR.set(Relay.Value.kOff);

		jevoisSerial = new SerialPort(115200, Port.kUSB);
	}
}
