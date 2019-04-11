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
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
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

	public static final boolean CLIMBER_ENABLED = false;
	
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

	final static int CLIMBER_TALON = 11;
	final static int CLIMBER_VICTOR = 21;

	// final static int BACK_LEFT_SENSOR = 0;
	final static int OUTER_LEFT_SENSOR = 1;
	final static int INNER_LEFT_SENSOR = 2;
	
	final static int BACK_RIGHT_SENSOR = 4;
	final static int OUTER_RIGHT_SENSOR = 5;
	final static int INNER_RIGHT_SENSOR = 3;

	final static int INTAKE_ARM_TALON = 10;
	final static int INTAKE_ARM_VICTOR = 20;
	final static int INTAKE_ROLLER_TALON = 12;
	final static int INTAKE_LIMIT_SWITCH = 0;
	//final static int INTAKE_POT = 2;

	final static int ELEVATOR_SPARK_1 = 7;
	final static int ELEVATOR_SPARK_2 = 8;

	final static int LEFT_OUTTAKE_TALON = 14;
	final static int RIGHT_OUTTAKE_TALON = 13;

	final static int LEFT_OUTTAKE_COUNTER = 6;
	final static int LEFT_OUTTAKE_LIMIT = 9;
	final static int RIGHT_OUTTAKE_COUNTER = 7;
	final static int RIGHT_OUTTAKE_LIMIT = 8;

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

	public static TalonSRX climberTalon = new TalonSRX(CLIMBER_TALON);
	public static VictorSPX climberVictor = new VictorSPX(CLIMBER_VICTOR);

	public static TalonSRX leftOuttakeTalon = new TalonSRX(LEFT_OUTTAKE_TALON);
	public static TalonSRX rightOuttakeTalon = new TalonSRX(RIGHT_OUTTAKE_TALON);
	/**
	 * Creating Gyro object
	 */	
	public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	public static final DigitalInput intakeLimitSwitch = new DigitalInput(INTAKE_LIMIT_SWITCH);
	public static SerialPort jevoisSerial;

	public static final Counter leftOuttakeCounter = new Counter(new DigitalInput(LEFT_OUTTAKE_COUNTER));
	public static final Counter rightOuttakeCounter = new Counter(new DigitalInput(RIGHT_OUTTAKE_COUNTER));

	public static final DigitalInput leftOuttakeLimit = new DigitalInput(LEFT_OUTTAKE_LIMIT);
	public static final DigitalInput rightOuttakeLimit = new DigitalInput(RIGHT_OUTTAKE_LIMIT);
		
	
	//LED lights
	public static Relay leftLED_PB = new Relay(LEFT_PB_RELAY, Relay.Direction.kBoth);
	public static Relay leftLED_GB = new Relay(LEFT_GR_RELAY, Relay.Direction.kBoth);
	public static Relay rightLED_PB = new Relay(RIGHT_PB_RELAY, Relay.Direction.kBoth);
	public static Relay rightLED_GR = new Relay(RIGHT_GR_RELAY, Relay.Direction.kBoth);

	//Creating sensor objects
	public static TapeSensor backLeftSensor;
	public static TapeSensor outerLeftSensor;
	public static TapeSensor innerLeftSensor;
	
	public static TapeSensor backRightSensor;
	public static TapeSensor outerRightSensor;
	public static TapeSensor innerRightSensor;
	
	public static void init() { //r/outoftheloop
		// System.out.println("BL:"+BACK_LEFT_SENSOR);
		// backLeftSensor = new TapeSensor(BACK_LEFT_SENSOR);
		System.out.println("OL:"+OUTER_LEFT_SENSOR);
		outerLeftSensor = new TapeSensor(OUTER_LEFT_SENSOR);
		System.out.println("IL:"+INNER_LEFT_SENSOR);
		innerLeftSensor = new TapeSensor(INNER_LEFT_SENSOR);
		System.out.println("BR:"+BACK_RIGHT_SENSOR);
		backRightSensor = new TapeSensor(BACK_RIGHT_SENSOR);
		System.out.println("OR:"+OUTER_RIGHT_SENSOR);
		outerRightSensor = new TapeSensor(OUTER_RIGHT_SENSOR);
		System.out.println("IR:"+INNER_RIGHT_SENSOR);
		innerRightSensor = new TapeSensor(INNER_RIGHT_SENSOR);

		// leftDriveSpark1.setInverted(true);
		// rightDriveSpark1.setInverted(true);
		//Set followers
		rightDriveSpark2.follow(rightDriveSpark1);
		rightDriveSpark3.follow(rightDriveSpark1);
		leftDriveSpark2.follow(leftDriveSpark1);
		leftDriveSpark3.follow(leftDriveSpark1);

		leftDriveSpark1.setOpenLoopRampRate(0.25);
		rightDriveSpark1.setOpenLoopRampRate(0.25); 
		
		//	Ratio of wheel rotations to encoder rotations
		//	Also equal to the ratio of teeth on the motor gears to teeth on the wheel gears
		double gearRatio = 14.0 / 50.0;
		//	1 rotation has 2π radians
		double rotationsToRadians = 2 * Math.PI;
		//	The wheels have radius 3"
		double wheelRadius = 6;
		//	Conversion of encoder rotations to linear distance travelled by the robot (in inches)
		//		Multiplying by gear ratio gives the number of wheel rotations
		//		Multiplying by 2π gives the number of radians rotated by the wheels
		//		Multiplying by wheel radius gives the distance travelled by the robot (in inches)
		double rotationsToInches = gearRatio * wheelRadius;


		leftDriveSpark1.getEncoder().setPositionConversionFactor(rotationsToInches);
		leftDriveSpark1.getEncoder().setVelocityConversionFactor(rotationsToInches);
		rightDriveSpark1.getEncoder().setPositionConversionFactor(rotationsToInches);
		rightDriveSpark1.getEncoder().setVelocityConversionFactor(rotationsToInches);
		
		leftDriveSpark2.getEncoder().setPositionConversionFactor(rotationsToInches);
		leftDriveSpark2.getEncoder().setVelocityConversionFactor(rotationsToInches);
		rightDriveSpark2.getEncoder().setPositionConversionFactor(rotationsToInches);
		rightDriveSpark2.getEncoder().setVelocityConversionFactor(rotationsToInches);

		intakeArmTalon.configOpenloopRamp(0.1);
		intakeArmTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		intakeArmTalon.setSensorPhase(false);
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
		if (CLIMBER_ENABLED) {
			climberTalon.set(ControlMode.PercentOutput,0.0);
			climberVictor.follow(climberTalon);
		}


		//LEDs off by default:
		leftLED_PB.set(Relay.Value.kOff);
		leftLED_GB.set(Relay.Value.kOff);
		rightLED_PB.set(Relay.Value.kOff);
		rightLED_GR.set(Relay.Value.kOff);

		leftOuttakeTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		leftOuttakeTalon.setSensorPhase(false);
		leftOuttakeTalon.config_kP(0, 8.0, 0);
		leftOuttakeTalon.config_kI(0, 0.0, 0);
		leftOuttakeTalon.config_kD(0, 0.001, 0);

		
		rightOuttakeTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		rightOuttakeTalon.setSensorPhase(true);
		rightOuttakeTalon.config_kP(0, 10.0, 0);
		rightOuttakeTalon.config_kI(0, 0.0, 0);
		rightOuttakeTalon.config_kD(0, 0.001, 0);

		rightOuttakeTalon.set(ControlMode.PercentOutput, 0);

		climberTalon.configSelectedFeedbackSensor(FeedbackDevice.Analog);

		// jevoisSerial = new SerialPort(115200, Port.kUSB);

	}
}
