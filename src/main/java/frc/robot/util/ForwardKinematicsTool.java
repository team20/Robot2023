package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

public class ForwardKinematicsTool {
	public static double[] getArmPosition(double lowerArmAngleDeg, double upperArmAngleDeg) {
		double lowerArmAngle = Math.toRadians(lowerArmAngleDeg);
		double upperArmAngle = Math.toRadians(upperArmAngleDeg);
		// Do elbow up when x < 180 degrees, elbow down when x > 180 degrees
		boolean isElbowUp = upperArmAngleDeg < 180;
		// Find hypotenuse using law of cosines
		double hypotenuse = Math
				.sqrt(Math.pow(ArmConstants.kLowerArmLength, 2) + Math.pow(ArmConstants.kUpperArmLength, 2)
						- 2 * ArmConstants.kLowerArmLength * ArmConstants.kUpperArmLength * Math.cos(upperArmAngle));
		// find angle d: btwn klowerArmLength and hypotenuse using law of cosines
		double d = Math.acos((Math.pow(ArmConstants.kUpperArmLength, 2) - Math.pow(ArmConstants.kLowerArmLength, 2)
				- Math.pow(hypotenuse, 2)) / (-2 * ArmConstants.kLowerArmLength * hypotenuse));

		// find angle f: btwn hypotenuse and x length
		double f;
		if (isElbowUp) {
			f = lowerArmAngle - d;
		} else {
			f = lowerArmAngle + d;
		}
		double y = hypotenuse * Math.sin(f);
		double x = hypotenuse * Math.cos(f);
		double[] position = { x, y };
		return position;
	}
}
