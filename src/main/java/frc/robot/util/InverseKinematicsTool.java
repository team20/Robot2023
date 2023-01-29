// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class InverseKinematicsTool {
	public static Double[] getArmAngles(double x, double y) {
		// IMPORTANT NOTE: "HYPOTENUSE" refers to the line segment formed between arm
		// base, and the point we want to go to. To avoid repeating that, HYPOTENUSE
		// will be used to refer to that line segment
		// All calculations are in radians
		// A reference position must be known to calculate the X and Y vectors for the
		// entire arm, which is why reference positions are need
		double referenceLowerArmVectorX = ArmConstants.kLowerArmLength * Math.cos(Math.toRadians(90));
		double referenceLowerArmVectorY = ArmConstants.kLowerArmLength * Math.sin(Math.toRadians(90));
		double referenceUpperArmVectorX = ArmConstants.kUpperArmLength * Math.cos(Math.toRadians(0));
		double referenceUpperArmVectorY = ArmConstants.kUpperArmLength * Math.sin(Math.toRadians(0));
		// If you had coordinates where (0, 0) was the base of the arm, you could use
		// those numbers as the target vector values. (14, 15) would mean
		// targetCombinedArmVectorX would be 14, and targetCombinedArmVectorY would be
		// 15
		double targetCombinedArmVectorX = referenceLowerArmVectorX + referenceUpperArmVectorX + x;
		double targetCombinedArmVectorY = referenceLowerArmVectorY + referenceUpperArmVectorY + y;
		// This finds the angle between the arm base(a horizontal line,) and the
		// HYPOTENUSE
		double targetCombinedArmAngle = Math.atan2(targetCombinedArmVectorY, targetCombinedArmVectorX);
		// These next three variables are part of the cosine rule
		double hypotenuseSquared = Math.pow(targetCombinedArmVectorX, 2) + Math.pow(targetCombinedArmVectorY, 2);
		double lowerArmLengthSquared = Math.pow(ArmConstants.kLowerArmLength, 2);
		double upperArmLengthSquared = Math.pow(ArmConstants.kUpperArmLength, 2);
		// This uses the cosine rule to get the angle between the two arms
		double targetAngleBetweenLowerAndUpperArm = Math
				.acos((lowerArmLengthSquared + upperArmLengthSquared - hypotenuseSquared)
						/ (2 * ArmConstants.kLowerArmLength * ArmConstants.kUpperArmLength));
		// This is the angle formed if you extended the lower arm, and you form an angle
		// on the extended side with the upper arm
		double targetAngleFormedByArms = Math.abs(-(Math.PI - targetAngleBetweenLowerAndUpperArm));
		// This finds the angle between the lower arm and the HYPOTENUSE
		double targetAngleFormedByLowerArmAndHypotenuse = Math.atan2(
				ArmConstants.kUpperArmLength * Math.sin(targetAngleFormedByArms),
				ArmConstants.kLowerArmLength + ArmConstants.kUpperArmLength * Math.cos(targetAngleFormedByArms));
		// This takes the angle between the arm base and the HYPOTENUSE, and adds the
		// angle between the HYPOTENUSE and the lower arm, giving us the angle between
		// the lower arm and the arm base
		double targetLowerArmAngle = targetCombinedArmAngle + targetAngleFormedByLowerArmAndHypotenuse;
		/*
		 * This works because the lower arm acts as a transversal between two parallel
		 * lines(both of which are horziontal.) Since the encoders are calibrated to
		 * 0 degrees being flat, and targetLowerArmAngle is relative to a horziontal
		 * line, this means targetLowerArmAngle has a corresponding angle, which are
		 * congruent to each other. This means subtracting targetAngleFormedByArms from
		 * targetLowerArmAngle leaves us with the angle of the arm relative to a flat
		 * line, which is the angle needed for the encoders to work. If the arm goes
		 * below the flat line, a negative angle is produced, but 360 can be added to it
		 * to make it work with 0-360 mode
		 */
		double targetUpperArmAngle = targetLowerArmAngle - targetAngleFormedByArms;
		Double[] returnValue = { targetLowerArmAngle, targetUpperArmAngle };
		if (Double.isNaN(targetLowerArmAngle) || Double.isNaN(targetUpperArmAngle)) {
			try {
				throw new Exception("Target position unreachable");
			} catch (Exception e) {
			}
			returnValue[0] = null;
			returnValue[1] = null;
		}
		return returnValue;
	}
}
