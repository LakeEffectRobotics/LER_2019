package frc.robot;

public class Tools {
	public static double fitToRange(double value, double min, double max) {
		value = value < min ? min : value;
		value = value > max ? max : value;
		return value;
	}
	public static double setAbsoluteMinimum(double value, double min) {
		value = (value < min && value > 0) ? min : value;
		value = (value > -min && value < 0) ? -min : value;
		return value;
	}
	public static double getTimeSeconds() {
		return (double) System.currentTimeMillis() / 1000.0;
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
}
