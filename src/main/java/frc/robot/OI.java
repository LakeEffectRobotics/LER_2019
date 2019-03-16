/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.Disco;
import frc.robot.commands.HoldOuttake;
import frc.robot.commands.StraightGyroDriveCommand;
import frc.robot.commands.autonomous.AutoIntakeCommand;
import frc.robot.commands.autonomous.VisionDriveCommand;
import frc.robot.commands.instant.SetElevatorHeightCommand;
import frc.robot.commands.instant.SetIntakeArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;

public class OI {
	
	// input number definitions
	final int L_JOY = 0;
	final int R_JOY = 1;
	
	final int XBOX = 2;

	//Left Joystick buttons
	final int TAPE_DRIVE_BUTTON = 3;
	final int GYRO_DRIVE_BUTTON = 1;

	//Right Joystick buttons
	final int VISION_DRIVE_BUTTON = 3;
	final int SLOW_DRIVE_BUTTON = 2;
	final int SHAWN_DRIVE_BUTTON = 1;

	//Xbox buttons:
	final int GROUND_HEIGHT_BUTTON = XBoxController.XBOX_A;
	final int LOW_ROCKET_BUTTON = XBoxController.XBOX_X;
	final int MID_ROCKET_BUTTON = XBoxController.XBOX_Y;
	final int HIGH_ROCKET_BUTTON = XBoxController.XBOX_B;
	final int CARGO_SHIP_HEIGHT_BUTTON = XBoxController.XBOX_RB;

	final int ELEVATOR_SHIFT_UP = XBoxController.XBOX_LEFT_TRIGGER;
	final int ELEVATOR_SHIFT_DOWN = XBoxController.XBOX_RIGHT_TRIGGER;

	final int INTAKE_BUTTON = XBoxController.XBOX_START;
	final int INTAKE_CANCEL_BUTTON = XBoxController.XBOX_BACK;

	final int OUTTAKE_LEFT_BUTTON = XBoxController.XBOX_DPAD_LEFT_ANGLE;
	final int OUTTAKE_RIGHT_BUTTON = XBoxController.XBOX_DPAD_RIGHT_ANGLE;

	final int DISCO_BUTTON = XBoxController.XBOX_R3;

	//Initialisation
	public Joystick lJoy = new Joystick(L_JOY);
	public Joystick rJoy = new Joystick(R_JOY);
	public XBoxController xbox = new XBoxController(XBOX);

	JoystickButton tapeDrive = new JoystickButton(lJoy, TAPE_DRIVE_BUTTON);
	JoystickButton gyroDrive = new JoystickButton(lJoy, GYRO_DRIVE_BUTTON);

	JoystickButton visionDrive = new JoystickButton(rJoy, VISION_DRIVE_BUTTON);
	public JoystickButton slowDrive = new JoystickButton(rJoy, SLOW_DRIVE_BUTTON);
	public JoystickButton shawnDrive = new JoystickButton(rJoy, SHAWN_DRIVE_BUTTON);

	XBoxButton groundHeight = new XBoxButton(xbox, GROUND_HEIGHT_BUTTON);
	XBoxButton lowHeight = new XBoxButton(xbox, LOW_ROCKET_BUTTON);
	XBoxButton midHeight = new XBoxButton(xbox, MID_ROCKET_BUTTON);
	XBoxButton highHeight = new XBoxButton(xbox, HIGH_ROCKET_BUTTON);
	XBoxButton cargoShipHeight = new XBoxButton(xbox, CARGO_SHIP_HEIGHT_BUTTON);
	
	XBoxButton intake = new XBoxButton(xbox, INTAKE_BUTTON);
	XBoxButton cancelIntake = new XBoxButton(xbox, INTAKE_CANCEL_BUTTON);
	
	XBoxButton outtakeLeft = new XBoxButton(xbox, OUTTAKE_LEFT_BUTTON);
	XBoxButton outtakeRight = new XBoxButton(xbox, OUTTAKE_RIGHT_BUTTON);

	public XBoxTrigger shiftUp = new XBoxTrigger(xbox, true);
	public XBoxTrigger shiftDown = new XBoxTrigger(xbox, false);

	XBoxButton disco = new XBoxButton(xbox, DISCO_BUTTON);

	public void init() {
		//Joystick Buttons
		gyroDrive.whileHeld(new StraightGyroDriveCommand());
		visionDrive.whileHeld(new VisionDriveCommand());

		//XBox Buttons
		groundHeight.whenPressed(new SetElevatorHeightCommand(Elevator.GROUND_HEIGHT));
		lowHeight.whenPressed(new SetElevatorHeightCommand(Elevator.LOW_HEIGHT));
		midHeight.whenPressed(new SetElevatorHeightCommand(Elevator.MID_HEIGHT));
		highHeight.whenPressed(new SetElevatorHeightCommand(Elevator.HIGH_HEIGHT));
		cargoShipHeight.whenPressed(new SetElevatorHeightCommand(Elevator.CARGO_SHIP_HEIGHT));

		intake.whenPressed(new AutoIntakeCommand());
		cancelIntake.whenPressed(new SetIntakeArm(Intake.POSITION_UP));

		outtakeLeft.whenPressed(new HoldOuttake(Outtake.SIDE_LEFT, Outtake.L_OUT));
		outtakeLeft.whenReleased(new HoldOuttake(Outtake.SIDE_LEFT, Outtake.L_IN));
		outtakeRight.whenPressed(new HoldOuttake(Outtake.SIDE_RIGHT, Outtake.R_OUT));
		outtakeRight.whenReleased(new HoldOuttake(Outtake.SIDE_RIGHT, Outtake.R_IN));

		disco.whileHeld(new Disco());
	}
}
