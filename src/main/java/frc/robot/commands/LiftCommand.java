package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LiftCommand extends Command {
	final double MULTIPLIER = 12;
	
    public LiftCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.lift);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        int targetPosition = RobotMap.liftTalon.getTargetPosition();
        if (Robot.oi.highLiftButton.get()) {
            targetPosition = Lift.ROCKET_PORT_HEIGHT_3; //  Or should this be Lift.ROCKET_HATCH_HEIGHT_3?
        }
        /*
        TODO: Recreate the 2018 code with this year's scoring heights
        --------------------------------------------------------------
    	if (Robot.oi.scale_height_button.get()) {
    		target_position = Lift.SCALE_HEIGHT;
    	}
    	else if (Robot.oi.switch_height_button.get()) {
    		target_position = Lift.SWITCH_HEIGHT;
    	}
    	else if (Robot.oi.ground_height_button.get()) {
    		target_position = Lift.GROUND_HEIGHT;
        }
    	target_position += Robot.oi.xbox.getJoyRightY() * MULTIPLIER;
        --------------------------------------------------------------
        final static double ROCKET_PORT_HEIGHT_1 = 2708;
        final static double ROCKET_PORT_HEIGHT_2 = 2708;
        final static double ROCKET_PORT_HEIGHT_3 = 2708;
        final static double SHIP_PORT_HEIGHT = 2708;

        final static double ROCKET_HATCH_HEIGHT_1 = 2708;
        final static double ROCKET_HATCH_HEIGHT_2 = 2708;
        final static double ROCKET_HATCH_HEIGHT_3 = 2708;
        final static double SHIP_HATCH_HEIGHT = ROCKET_HATCH_HEIGHT_1;

        final static double GROUND_HEIGHT = 2708;
        */
    	RobotMap.liftTalon.setTargetPosition(targetPosition);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
