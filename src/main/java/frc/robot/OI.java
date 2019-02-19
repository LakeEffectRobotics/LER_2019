/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.StraightGyroDriveCommand;
import frc.robot.commands.instant.SetShawnDriveCommand;
import frc.robot.commands.LockDriveCommand;
// import frc.robot.commands.instant.ResetLiftPositionCommand;
// import frc.robot.commands.instant.ToggleClawPositionCommand;
// import frc.robot.commands.instant.IntakeOpenCommand;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.XBoxController;
import frc.robot.XBoxPOVButton;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

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
	public Button toggleShawnDriveButton = new JoystickButton(l_joy, TOGGLE_SHAWN_DRIVE);
	
	public Button holdShawnDriveButton = new JoystickButton(r_joy, HOLD_SHAWN_DRIVE);

	public Button lockDriveButton = new JoystickButton(r_joy, LOCK_DRIVE);
	// -- public Button buttonName = new JoystickButton(controller, BUTTON_NAME);

	public void init() {
		//Inithalise event handling
		toggleShawnDriveButton.whenPressed(new SetShawnDriveCommand(SetShawnDriveCommand.Mode.TOGGLE));

		holdShawnDriveButton.whenPressed(new SetShawnDriveCommand(SetShawnDriveCommand.Mode.ON));
		holdShawnDriveButton.whenReleased(new SetShawnDriveCommand(SetShawnDriveCommand.Mode.OFF));

		lockDriveButton.whileHeld(new LockDriveCommand());
		// -- buttonName.whenHeld(new Command())
	}
}
