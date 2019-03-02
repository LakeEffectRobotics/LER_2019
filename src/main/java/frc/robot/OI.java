/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.HoldOuttake;
import frc.robot.commands.LockDriveCommand;
// import frc.robot.commands.instant.ResetLiftPositionCommand;
// import frc.robot.commands.instant.ToggleClawPositionCommand;
// import frc.robot.commands.instant.IntakeOpenCommand;
import frc.robot.commands.instant.BumpElevatorHeight;
import frc.robot.commands.instant.ElevatorToIntakeHeight;
import frc.robot.commands.instant.SetShawnDriveCommand;
import frc.robot.commands.instant.ToggleHatchMode;
import frc.robot.commands.instant.ToggleIntakeArm;
import frc.robot.subsystems.Outtake;

public class OI {
	
	// input number definitions
	final int L_JOY = 0;
	final int R_JOY = 1;
	
	final int XBOX = 2;

	// button number definitions
	
	// right joystick buttons	//  TODO: Update all controls
	final int TOGGLE_SHAWN_DRIVE = 1;
	final int HOLD_SHAWN_DRIVE = 2;
	final int LOCK_DRIVE = 3;
	// -- final int BUTTON_NAME = number;

	// left joystick buttons


	// controller initialization
	public Joystick l_joy = new Joystick(L_JOY);
	public Joystick r_joy = new Joystick(R_JOY);
	public XBoxController xbox = new XBoxController(XBOX);

	// button initialization 
	
	// can be changed to driver's preferences
	public Button toggleShawnDriveButton = new JoystickButton(l_joy, TOGGLE_SHAWN_DRIVE);
	//Lift controls
	public Button rStickLift = new XBoxButton(xbox, XBoxController.XBOX_RIGHT_Y);
	
	public Button highLiftButton = new XBoxButton(xbox, XBoxController.XBOX_Y);
	public Button mediumLiftButton = new XBoxButton(xbox, XBoxController.XBOX_X);
	public Button lowLiftButton = new XBoxButton(xbox, XBoxController.XBOX_A);
	public Button cargoIntakeButton = new XBoxButton(xbox, XBoxController.XBOX_RIGHT_TRIGGER);
	public Button cargoOuttakeButton = new XBoxButton(xbox, XBoxController.XBOX_LEFT_TRIGGER);
	public Button rightFlipperButton = new XBoxButton(xbox, XBoxController.XBOX_RB);
	public Button leftFlipperButton = new XBoxButton(xbox, XBoxController.XBOX_LB);
	



	public Button holdShawnDriveButton = new JoystickButton(r_joy, HOLD_SHAWN_DRIVE);

	public Button lockDriveButton = new JoystickButton(r_joy, LOCK_DRIVE);
	//XBox Button initialization

	//Start and Back
	public Button climberButton = new XBoxButton(xbox, XBoxController.XBOX_BACK);

	//Lettered Buttons
	public Button stepUpButton = new XBoxButton(xbox, XBoxController.XBOX_Y);
	public Button stepDownButton = new XBoxButton(xbox, XBoxController.XBOX_X);
	public Button intakeHeightButton = new XBoxButton(xbox, XBoxController.XBOX_A);
	public Button toggleIntakeArmButton = new XBoxButton(xbox, XBoxController.XBOX_B);

	//Bumpers and Triggers
	public Button toggleHatchModeButton = new XBoxButton(xbox, XBoxController.XBOX_L3);
	// public Button autoOuttakeButton = new XBoxButton(xbox, XBoxController.XBOX_LB);
	public Button hatchOffsetButton = new XBoxButton(xbox, XBoxController.XBOX_RIGHT_TRIGGER);
	public Button intakeButton = new XBoxButton(xbox, XBoxController.XBOX_LEFT_TRIGGER);

	//D-pad
	public Button leftOuttake = new XBoxButton(xbox, XBoxController.XBOX_LB);
	public Button rightOuttake = new XBoxButton(xbox, XBoxController.XBOX_RB);
	


	// -- public Button buttonName = new JoystickButton(controller, BUTTON_NAME);

	public void init() {
		//Initialize event handling

		//Joystick Buttons
		toggleShawnDriveButton.whenPressed(new SetShawnDriveCommand(SetShawnDriveCommand.Mode.TOGGLE));
		holdShawnDriveButton.whenPressed(new SetShawnDriveCommand(SetShawnDriveCommand.Mode.ON));
		holdShawnDriveButton.whenReleased(new SetShawnDriveCommand(SetShawnDriveCommand.Mode.OFF));
		lockDriveButton.whileHeld(new LockDriveCommand());

		//XBox Buttons
		stepUpButton.whenPressed(new BumpElevatorHeight(BumpElevatorHeight.UP));
		stepDownButton.whenPressed(new BumpElevatorHeight(BumpElevatorHeight.DOWN));
		intakeHeightButton.whenPressed(new ElevatorToIntakeHeight());
		toggleIntakeArmButton.whenPressed(new ToggleIntakeArm());
		toggleHatchModeButton.whenPressed(new ToggleHatchMode());
		leftOuttake.whileHeld(new HoldOuttake(Outtake.LEFT));
		rightOuttake.whileHeld(new HoldOuttake(Outtake.RIGHT));
		
		// -- buttonName.whenHeld(new Command())
	}
}
