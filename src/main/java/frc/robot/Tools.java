package frc.robot;

public class Tools {
	//	Ratio of wheel rotations to encoder rotations
	//	Also equal to the ratio of teeth on the motor gears to teeth on the wheel gears
	final static double GEAR_RATIO = 14 / 50;
	//	1 rotation has 2π radians
	final static double ROTATIONS_TO_RADIANS = 2 * Math.PI;
	//	The wheels have radius 3"
	final static double WHEEL_RADIUS = 3;
	//	Conversion of encoder rotations to linear distance travelled by the robot (in inches)
	//		Multiplying by gear ratio gives the number of wheel rotations
	//		Multiplying by 2π gives the number of radians rotated by the wheels
	//		Multiplying by wheel radius gives the distance travelled by the robot (in inches)
	final static double ROTATIONS_TO_INCHES = GEAR_RATIO * ROTATIONS_TO_RADIANS * WHEEL_RADIUS;

	public static double fitToRange(double value, double min, double max) {
		value = value < min ? min : value;
		value = value > max ? max : value;
		return value;
	}
	public static double forceMaximum(double value, double max) {
		return fitToRange(value, -max, max);
	}
	public static double getTimeSeconds() {
		return (double) System.currentTimeMillis() / 1000d;
	}
	public static double closestEquivalentAngle(double target_angle) {
		double t = target_angle;
		double c = Robot.gyro.getAbsoluteAngle();
		boolean b = true;
		while (b) {
			if (Math.abs(c - t) > Math.abs(c - (t + 360))) {
	    		t += 360;
	    	}
			else if (Math.abs(c - t) > Math.abs(c - (t - 360))) {
				t -= 360;
			}
			else {
				b = false;
			}
		}
		return t;
	}
	public static double rotationsToInches(double rotations) {
		return rotations * ROTATIONS_TO_INCHES;
	}
	public static double inchesToRotations(double inches) {
		return inches / ROTATIONS_TO_INCHES;
	}
}
