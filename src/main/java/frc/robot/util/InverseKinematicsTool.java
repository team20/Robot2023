// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import frc.robot.Constants.ArmConstants;

/**
 * Contains the inverse kinematics code to calculate the arm angles. That's
 * literally all it does, it contains one method to do inverse kinematics to
 * determine the angles the arms should at.
 * <p>
 * IMPORTANT NOTES/DEFINTIONS:
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
	public static Double[] calculateArmAngles(double x, double y) {
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
		double upperArmAngle = Math.acos((lowerArmLengthSquared + upperArmLengthSquared - hypotenuseSquared)
				/ (2 * ArmConstants.kLowerArmLength * ArmConstants.kUpperArmLength));
		// This is the angle formed if you extended the lower arm, and you form an angle
		// on the extended side of the lower arm with the upper arm
		double angleFormedByArms;
		// If some random condition, elbow up, else, elbow down
		if (true) {
			angleFormedByArms = Math.abs(-(Math.PI - upperArmAngle));
		} else {
			angleFormedByArms = Math.PI - upperArmAngle;
		}
		// This finds the angle between the lower arm and the HYPOTENUSE
		double angleFormedByLowerArmAndHypotenuse = Math.atan2(
				ArmConstants.kUpperArmLength * Math.sin(angleFormedByArms),
				ArmConstants.kLowerArmLength + ArmConstants.kUpperArmLength * Math.cos(angleFormedByArms));
		// This takes the angle between the arm base and the HYPOTENUSE, and adds the
		// angle between the HYPOTENUSE and the lower arm, giving us the angle between
		// the lower arm and the arm base
		double lowerArmAngle;
		if (true) {
			lowerArmAngle = combinedArmAngle + angleFormedByLowerArmAndHypotenuse;
		} else {
			lowerArmAngle = combinedArmAngle - angleFormedByLowerArmAndHypotenuse;
		}
		// Convert to degrees and a Double for NaN checks
		Double[] returnValue = { Math.toDegrees(lowerArmAngle), Math.toDegrees(upperArmAngle) };
		// If the position is invalid, the code will output NaN for at least one of the
		// angles. If that's the case, log an error, and set the returned angle array to
		// null
		if (Double.isNaN(lowerArmAngle) || Double.isNaN(upperArmAngle)) {
			try {
				throw new Exception("Target position unreachable");
			} catch (Exception e) {
				System.out.println("Target position unreachable");
			}
			returnValue = null;
		}
		return returnValue;
	}
}
