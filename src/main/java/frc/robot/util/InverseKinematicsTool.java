// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class InverseKinematicsTool {
	public static Double[] getArmAngles(double x, double y) {
		// cos(lowerArmVectorX) = angle of the lower joint/the length of the lower arm
		// All in radians
		double referenceLowerArmVectorX = ArmConstants.kLowerArmLength * Math.cos(Math.toRadians(90));
		double referenceLowerArmVectorY = ArmConstants.kLowerArmLength * Math.sin(Math.toRadians(90));
		double referenceUpperArmVectorX = ArmConstants.kUpperArmLength * Math.cos(Math.toRadians(0));
		double referenceUpperArmVectorY = ArmConstants.kUpperArmLength * Math.sin(Math.toRadians(0));

		double targetCombinedArmVectorX = referenceLowerArmVectorX + referenceUpperArmVectorX + x;
		double targetCombinedArmVectorY = referenceLowerArmVectorY + referenceUpperArmVectorY + y;
		double targetCombinedArmAngle = Math.atan2(targetCombinedArmVectorY, targetCombinedArmVectorX);
		double hypotenuseSquared = Math.pow(targetCombinedArmVectorX, 2) + Math.pow(targetCombinedArmVectorY, 2);
		double lowerArmLengthSquared = Math.pow(ArmConstants.kLowerArmLength, 2);
		double upperArmLengthSquared = Math.pow(ArmConstants.kUpperArmLength, 2);
		double cosAlpha = (lowerArmLengthSquared + upperArmLengthSquared - hypotenuseSquared)
				/ (2 * ArmConstants.kLowerArmLength * ArmConstants.kUpperArmLength);
		// This is the angle formed if you extended the lower arm, and you form an angle
		// on the extended side with the upper arm
		double targetAngleFormedByArms = Math.abs(-(Math.PI - Math.acos(cosAlpha)));
		double targetAngleFormedByLowerArmAndHypotenuse = Math.atan2(
				ArmConstants.kUpperArmLength * Math.sin(targetAngleFormedByArms),
				ArmConstants.kLowerArmLength + ArmConstants.kUpperArmLength * Math.cos(targetAngleFormedByArms));
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
		Double[] returnValue = {targetLowerArmAngle, targetUpperArmAngle};
		if(Double.isNaN(targetLowerArmAngle) || Double.isNaN(targetUpperArmAngle)){
			try{
				throw new Exception("Target position unreachable");
			}catch(Exception e){

			}
			returnValue[0] = null;
			returnValue[1] = null;
		}
		return returnValue;
	}
}
