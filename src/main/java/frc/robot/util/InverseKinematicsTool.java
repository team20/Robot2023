// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import frc.robot.Constants.ArmConstants;

/**
 * Contains the inverse kinematics code to calculate the arm angles. That's
 * literally all it does, it contains one method to do inverse kinematics to
 * determine the angles the arms should at.
 */
public class InverseKinematicsTool {
	/**
	 * Takes a position relative to the lower arm motor, and returns the angles for
	 * the two arms
	 * The position is relative to the lower arm motor(a.k.a, the origin is where
	 * the lower arm motor is) because that's where the arm starts, so it makes
	 * sense for the lower arm motor to be the origin
	 * 
	 * @param x
	 *          The number of units to move the arm horizontally from the reference
	 *          position
	 * @param y
	 *          The number of units to move the arm vertically from the reference
	 *          position
	 * @return
	 *         The angle of the lower arm and the angle of the upper arm in degrees
	 */
	public static Double[] getArmAngles(double x, double y) {
		// IMPORTANT NOTE: "HYPOTENUSE" refers to the line segment formed between the
		// arm base, and the point we want to go to. To avoid repeating that, HYPOTENUSE
		// will be used to refer to that line segment
		// All calculations are in radians
		// This finds the angle between the arm base(a horizontal line,) and the
		// HYPOTENUSE
		double targetCombinedArmAngle = Math.atan2(y, x);
		// These next three variables are part of the cosine rule
		double hypotenuseSquared = Math.pow(x, 2) + Math.pow(y, 2);
		double lowerArmLengthSquared = Math.pow(ArmConstants.kLowerArmLength, 2);
		double upperArmLengthSquared = Math.pow(ArmConstants.kUpperArmLength, 2);
		/*
		 * This uses the cosine rule to get the angle between the two arms, which is the
		 * angle we want the upper to be at. The reason for this is that the upper arm
		 * motor moves with the lower arm, and the angle of the upper arm will be
		 * relative to the lower arm. Therefore, the angle between the upper and lower
		 * arm is the angle we want to give to our PIDs
		 */
		double targetUpperArmAngle = Math
				.acos((lowerArmLengthSquared + upperArmLengthSquared - hypotenuseSquared)
						/ (2 * ArmConstants.kLowerArmLength * ArmConstants.kUpperArmLength));
		// This is the angle formed if you extended the lower arm, and you form an angle
		// on the extended side of the lower arm with the upper arm
		double targetAngleFormedByArms = Math.abs(-(Math.PI - targetUpperArmAngle));
		// This finds the angle between the lower arm and the HYPOTENUSE
		double targetAngleFormedByLowerArmAndHypotenuse = Math.atan2(
				ArmConstants.kUpperArmLength * Math.sin(targetAngleFormedByArms),
				ArmConstants.kLowerArmLength + ArmConstants.kUpperArmLength * Math.cos(targetAngleFormedByArms));
		// This takes the angle between the arm base and the HYPOTENUSE, and adds the
		// angle between the HYPOTENUSE and the lower arm, giving us the angle between
		// the lower arm and the arm base
		double targetLowerArmAngle = targetCombinedArmAngle + targetAngleFormedByLowerArmAndHypotenuse;
		// Convert to degrees and a Double for NaN checks
		Double[] returnValue = { Math.toDegrees(targetLowerArmAngle), Math.toDegrees(targetUpperArmAngle) };
		// If the position is invalid, the code will output NaN for at least one of the
		// angles. If that's the case, log an error, and set the returned angle array to
		// null
		if (Double.isNaN(targetLowerArmAngle) || Double.isNaN(targetUpperArmAngle)) {
			try {
				throw new Exception("Target position unreachable");
			} catch (Exception e) {
			}
			returnValue = null;
		}
		return returnValue;
	}
}
