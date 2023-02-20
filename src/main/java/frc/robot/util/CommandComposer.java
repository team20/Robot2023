// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmScoreCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;

/**
 * CommandComposer allows you to use additional logic to determine what commmand
 * should be executed when a Trigger is true. To do this, create a static method
 * that returns a Command, and put in logic to determine what Command to execute
 * and with what arguements
 */
public class CommandComposer {
	public static Command createArmScoreCommand(ArmPosition armPosition) {
		double[] coordinates = ForwardKinematicsTool.getArmPosition(ArmSubsystem.get().getLowerArmAngle(),
				ArmSubsystem.get().getUpperArmAngle());
		boolean isArmForwards = coordinates[0] > 0;
		// If the arm is fowards, and we are only moving the arm between the foward
		// positions(not flipping over,) just return the command as normal
		if (isArmForwards && armPosition != ArmPosition.MEDIUM_BACK) {
			return new ArmScoreCommand(armPosition);
			// If the arm is backwards, and we want to move the arm fowards, go to the
			// intermediate position first
		} else if (!isArmForwards && armPosition != ArmPosition.MEDIUM_BACK) {
			return new SequentialCommandGroup(new ArmScoreCommand(ArmPosition.INTERMEDIATE),
					new ArmScoreCommand(armPosition));
			// If we want to move the arm to flip back,
		} else if (armPosition == ArmPosition.MEDIUM_BACK) {
			// If the arm is fowards, go to the intermediate position first, then make the
			// arm flip back
			if (isArmForwards) {
				return new SequentialCommandGroup(new ArmScoreCommand(ArmPosition.INTERMEDIATE),
						new ArmScoreCommand(armPosition));
				// Otherwise, the arm is backwards, and we can just go directly to the backwards
				// medium position
			} else {
				return new ArmScoreCommand(armPosition);
			}
		}
		// Probably unreachable
		return null;
	}
}
