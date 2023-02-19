package frc.robot.util;

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
		// find angle btwn klowerArmLength and hypotenuse using law of cosines
		double angleBetweenLowerArmAndHypotenuse = Math
				.acos((Math.pow(ArmConstants.kUpperArmLength, 2) - Math.pow(ArmConstants.kLowerArmLength, 2)
						- Math.pow(hypotenuse, 2)) / (-2 * ArmConstants.kLowerArmLength * hypotenuse));

		// find angle btwn hypotenuse and x length
		double combinedArmAngle;
		if (isElbowUp) {
			combinedArmAngle = lowerArmAngle - angleBetweenLowerArmAndHypotenuse;
		} else {
			combinedArmAngle = lowerArmAngle + angleBetweenLowerArmAndHypotenuse;
		}
		double y = hypotenuse * Math.sin(combinedArmAngle);
		double x = hypotenuse * Math.cos(combinedArmAngle);
		double[] position = { x, y };
		return position;
	}
}
