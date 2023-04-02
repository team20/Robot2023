// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import frc.robot.Constants.ArmConstants;

/**
 * Contains the inverse kinematics code to calculate the arm angles. That's
 * literally all it does, it contains one method to do inverse kinematics to
 * determine the angles the arms should be at.
 * <p>
 * IMPORTANT NOTES/DEFINITIONS:
 * <p>
 * ORIGIN is defined as the location of the lower arm motor. This is because
 * the lower arm motor is effectively where the arm starts, so it makes sense
 * for the lower arm motor to be the origin
 * <p>
 * HYPOTENUSE is defined as the line segment formed between the
 * ORIGIN and the tip of the arm.
 */
public class InverseKinematicsTool {
	/**
	 * <p>
	 * Takes a position relative to the lower arm motor(ORIGIN), and returns the
	 * angles for the upper and lower arms.
	 * 
	 * @param x
	 *          The horizontal distance between the ORIGIN and the tip of the
	 *          arm(x-coordinate)
	 * @param y
	 *          The vertical distance between the ORIGIN and the tip of the
	 *          arm(y-coordinate)
	 * @return
	 *         The angle of the lower arm and the angle of the upper arm in degrees
	 */
	// Source: https://www.youtube.com/watch?v=V4s4Vd2BLi4
	public static double[] calculateArmAngles(double x, double y) {
		// All calculations are in radians
		// This finds the angle between the arm base(a horizontal line,) and the
		// HYPOTENUSE
		double combinedArmAngle = Math.atan2(y, x);
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
		double angleBetweenUpperAndLowerArm = Math
				.acos((lowerArmLengthSquared + upperArmLengthSquared - hypotenuseSquared)
						/ (2 * ArmConstants.kLowerArmLength * ArmConstants.kUpperArmLength));
		// This is the angle formed if you extended the lower arm, and you form an angle
		// on the extended side of the lower arm with the upper arm
		double angleFormedByArms;
		// Do elbow up when x > 0, elbow down when x < 0
		boolean isElbowUp = x > 0;
		if (isElbowUp) {
			angleFormedByArms = Math.abs(Math.PI - angleBetweenUpperAndLowerArm);
		} else {
			angleFormedByArms = Math.PI - angleBetweenUpperAndLowerArm;
		}
		// This finds the angle between the lower arm and the HYPOTENUSE
		double angleFormedByLowerArmAndHypotenuse = Math.atan2(
				ArmConstants.kUpperArmLength * Math.sin(angleFormedByArms),
				ArmConstants.kLowerArmLength + ArmConstants.kUpperArmLength * Math.cos(angleFormedByArms));
		double lowerArmAngle;
		double upperArmAngle;
		if (isElbowUp) {
			// This takes the angle between the arm base and the HYPOTENUSE, and adds the
			// angle between the HYPOTENUSE and the lower arm, giving us the angle between
			// the lower arm and the arm base. In elbow up mode, the lower arm is above the
			// hypotenuse, which is why the angle between the HYPOTENUSE and lower arm is
			// added
			lowerArmAngle = combinedArmAngle + angleFormedByLowerArmAndHypotenuse;
			// If elbow up, the angle formed between the two arms is the angle we want
			upperArmAngle = angleBetweenUpperAndLowerArm;
		} else {
			// In elbow down mode, the lower arm is between the arm base and lower arm, so
			// to get the lower arm angle, the angle between the HYPOTENUSE and lower arm is
			// subtracted from combinedArmAngle instead of added
			lowerArmAngle = combinedArmAngle - angleFormedByLowerArmAndHypotenuse;
			// If elbow down, 180 degrees + the angle the upper arm forms with the extended
			// portion of the lower arm is the angle we want. 180 degrees is added because
			// the upper arm in elbow down mode is on the opposite side of the lower arm
			upperArmAngle = Math.PI + angleFormedByArms;
		}
		// Convert to degrees and a Double for NaN checks
		double[] armAngles = { Math.toDegrees(lowerArmAngle), Math.toDegrees(upperArmAngle) };
		// If the position is invalid, the code will output NaN for at least one of the
		// angles. If that's the case, log an error, and set the returned angle array to
		// null
		if (Double.isNaN(lowerArmAngle) || Double.isNaN(upperArmAngle)) {
			armAngles = null;
			// Prevent the lower arm from going more than 10 degrees behind vertical or
			// below 45 degrees
		} else if (armAngles[0] > ArmConstants.kLowerArmMaxAngle || armAngles[0] < ArmConstants.kLowerArmMinAngle) {
			armAngles = null;
			// Prevent the upper arm from going more than 270 degrees or less than 15
			// degrees relative to the lower arm
		} else if (armAngles[1] > ArmConstants.kUpperArmMaxAngle || armAngles[1] < ArmConstants.kUpperArmMinAngle) {
			armAngles = null;
		}
		return armAngles;
	}
}
