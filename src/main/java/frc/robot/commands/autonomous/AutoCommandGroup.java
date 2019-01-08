package frc.robot.commands.autonomous;

import frc.robot.Robot;
import frc.robot.Robot.AutonomousTarget;
//import frc.robot.RobotMap;
import frc.robot.commands.TimerCommand;
import frc.robot.commands.instant.Stop;

//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Main autonomous command group for auto.<p>
 * There are 3 modes for the first score. The FMS sends information which tells us whether we need to head to the opposite side first.<br>
 * <ul><b>Switch</b> scores the switch on the side closest to the scale. (Avoid other robots who may want to score from the close side.<br>
 * <b>Scale</b> scores on the side of the scale closest to switch.<br>
 * <b>Auto Line</b> Moves the robot past the Auto Line...<p>
 * </ul>
 * After this, if there is a secondary target selected, the robot attempts to score another time by picking up a cube and putting it in the desired location.
 * <p>
 * <i>Tech Info</i><p>
 * There is a variable called <b><i>turn</i></b> which is -1 if starting on the left, and +1 if starting on the right. All of the turns we perform in auto use <b><i>turn</i></b> * angle so we only have to program for one side of the field. This way if we start on the opposite side, then the robot performs mirror image movements.
 *@author Ewan and Tim
 *
 *
 */
public class AutoCommandGroup extends CommandGroup {
	/**
	 * Declare speed constants
	 */
	public static final double SLOW_SPEED = 0.3;
	public static final double MEDIUM_SPEED = 0.5;
	public static final double HIGH_SPEED = 0.7;

	public AutoCommandGroup() {
		addSequential(new Stop());
	}
}
