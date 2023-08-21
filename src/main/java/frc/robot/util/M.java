package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmScoreAutoCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.DriveTimeCommand;
import frc.robot.commands.drive.TurnRelativeCommand;
import frc.robot.commands.gripper.WheelGripperCommand;
import frc.robot.commands.gripper.WheelGripperCommand.WheelGripperPosition;

public class M {
	public static Command getTwoScoreRedAuto() {
		return new SequentialCommandGroup(
				new WheelGripperCommand(WheelGripperPosition.STOP),
				new SequentialCommandGroup(
						new ArmScoreAutoCommand(ArmScoreAutoCommand.ArmPosition.TO_BACK_INTERMEDIATE),
						new ArmScoreAutoCommand(ArmScoreAutoCommand.ArmPosition.HIGH_BACK)),
				getOuttakePieceCommand(),
				new ParallelRaceGroup(
						new SequentialCommandGroup(
								new ArmScoreAutoCommand(ArmScoreAutoCommand.ArmPosition.TO_FORWARD_INTERMEDIATE),
								getPickupPieceCommand()),
						new SequentialCommandGroup(
								new WaitCommand(0.25),
								new DriveDistanceCommand(5.2),
								new DriveTimeCommand(-0.25, 200))),
				new ParallelCommandGroup(
						new WheelGripperCommand(WheelGripperPosition.INTAKE_CUBE_W_SENSOR),
						new DriveTimeCommand(-1, 350)),
				new ParallelCommandGroup(
						new SequentialCommandGroup(
								new TurnRelativeCommand(0.25),
								new DriveDistanceCommand(-3.5),
								getAnvitaAuto()),
						new SequentialCommandGroup(
								new ArmScoreAutoCommand(ArmScoreAutoCommand.ArmPosition.TO_BACK_INTERMEDIATE),
								new ArmScoreAutoCommand(ArmScoreAutoCommand.ArmPosition.MEDIUM_BACK))),
				getOuttakePieceCommand(),
				new ArmScoreAutoCommand(ArmScoreAutoCommand.ArmPosition.TO_FORWARD_INTERMEDIATE),
				new ArmScoreAutoCommand(ArmScoreAutoCommand.ArmPosition.POCKET));
	}

	private static Command getPickupPieceCommand() {
		return null;
	}

	private static Command getAnvitaAuto() {
		return null;
	}

	private static Command getOuttakePieceCommand() {
		return null;
	}
}