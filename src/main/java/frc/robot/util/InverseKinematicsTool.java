// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

/**
 * Contains the inverse kinematics code to calculate the arm angles. That's
 * literally all it does, it contains one method to do inverse kinematics to
 * determine the angles the arms should at.
 */
public class InverseKinematicsTool {
	/**
	 * Takes a distance to move the arm horizontally and vertically from the
	 * reference position, and returns the angles for the two arms
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
		// A reference position must be known to calculate the X and Y vectors for the
		// entire arm, which is why reference positions are needed
		// The current reference position is the lower arm at 90 degrees(straight up),
		// and the upper arm at 0 degrees(straight out.) (0, 0) is the tip of this
		// reference position
		SmartDashboard.putNumber("x", x);
		SmartDashboard.putNumber("y", y);
		// double referenceLowerArmVectorX = ArmConstants.kLowerArmLength *
		// Math.cos(Math.toRadians(90));
		// double referenceLowerArmVectorY = ArmConstants.kLowerArmLength *
		// Math.sin(Math.toRadians(90));
		// double referenceUpperArmVectorX = ArmConstants.kUpperArmLength *
		// Math.cos(Math.toRadians(-90));
		// double referenceUpperArmVectorY = ArmConstants.kUpperArmLength *
		// Math.sin(Math.toRadians(-90));
		// Modification notes: If you had arm coordinates where (0, 0) was the base of
		// the arm, you could use those numbers as the target vector values. (14, 15)
		// would mean targetCombinedArmectorX would be 14, and targetCombinedArmVectorY
		// would be 15. a.k.a, you could use foward kinematics to calculate arm
		// position, meaning (0, 0) is your arm position, and your starting offsets
		// where your arm is
		// x is the number of units to move horizontally from the reference position.
		// -1 would mean move the arm back 1 unit
		// double targetCombinedArmVectorX = referenceLowerArmVectorX +
		// referenceUpperArmVectorX + x;
		// y is the number of units to move vertically from the reference position.
		// -1 would mean move the arm down 1 unit
		// double targetCombinedArmVectorY = referenceLowerArmVectorY +
		// referenceUpperArmVectorY + y;
		// This finds the angle between the arm base(a horizontal line,) and the
		// HYPOTENUSE
		double targetCombinedArmAngle = Math.atan2(y, x);
		// These next three variables are part of the cosine rule
		double hypotenuseSquared = Math.pow(x, 2) + Math.pow(y, 2);
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
		 * The arm has two parallel lines, the arm base, which the lower arm uses, and a
		 * line where the lower arm and upper arm meet. Both are horizontal. This math
		 * works because the lower arm acts as a transversal between two parallel
		 * lines(the arm base, and the line where the upper and lower arm meet.) Since
		 * the encoders are calibrated to 0 degrees being horizontal, and
		 * targetLowerArmAngle is relative to a horziontal line, this means
		 * targetLowerArmAngle has a corresponding angle, which are congruent to each
		 * other. This means subtracting targetAngleFormedByArms from
		 * targetLowerArmAngle leaves us with the angle of the arm relative to a flat
		 * line, which is the angle needed for the encoders to work. If the arm goes
		 * below the flat line, a negative angle is produced, but 360 can be added to it
		 * to make it work with 0-360 mode
		 */
		double targetUpperArmAngle = targetAngleBetweenLowerAndUpperArm;
		Double[] returnValue = { Math.toDegrees(targetLowerArmAngle), Math.toDegrees(targetUpperArmAngle) };
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
