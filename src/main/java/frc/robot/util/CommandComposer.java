// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmScoreCommand;
import frc.robot.commands.arm.ArmScoreCommand.ArmPosition;
import frc.robot.commands.drive.BalancePIDCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.DriveTimeCommand;
import frc.robot.commands.drive.TagAlignCommand;
import frc.robot.commands.drive.TurnCommand;
import frc.robot.commands.drive.TurnRelativeCommand;
import frc.robot.commands.gripper.WheelGripperCommand;
import frc.robot.commands.gripper.WheelGripperCommand.WheelGripperPosition;
import frc.robot.commands.util.DeferredCommand;
import frc.robot.commands.util.DeferredCommandAuto;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.ArmSubsystem;

/**
 * When you need to construct long command chains, you can put those chains in
 * here instead of clogging RobotContainer.
 * <p>
 * CommandComposer also allows you to use additional logic to determine what
 * command should be executed when a Trigger is true. To do this, create a
 * static method that returns a Command, and put in logic to determine what
 * Command to execute and with what arguments
 */
public class CommandComposer {
	/**
	 * Creates a command or command group to move the arm. The command or command
	 * group changes depending on where the arm is moving to and where the arm is
	 * 
	 * @param armPosition The position the arm should move to
	 * @return A command or command group that has all the necessary steps to move
	 *         the arm to the desired position
	 */
	public static Command createArmScoreCommand(ArmPosition armPosition) {
		double[] coordinates = ForwardKinematicsTool.getArmPosition(ArmSubsystem.get().getLowerArmAngle(),
				ArmSubsystem.get().getUpperArmAngle());
		boolean isArmForwards = coordinates[0] > 0;
		// If the arm is forwards, check if the arm is going to flip over, and move to
		// an intermediate position if necessary
		if (isArmForwards) {
			// If we are moving to another forwards position, just move to the position
			if (armPosition != ArmPosition.MEDIUM_BACK && armPosition != ArmPosition.HIGH_BACK) {
				return new ArmScoreCommand(armPosition);
				// If we are moving to a backwards position, we need to move to an intermediate
				// position first
			} else {
				return new SequentialCommandGroup(new ArmScoreCommand(ArmPosition.TO_BACK_INTERMEDIATE),
						new ArmScoreCommand(armPosition));
			}
			// If the arm is backwards, check if the arm is going to flip over, and move to
			// an intermediate position if necessary
		} else {
			// If we are moving to another backwards position, just move to the position
			if (armPosition == ArmPosition.MEDIUM_BACK || armPosition == ArmPosition.HIGH_BACK) {
				return new ArmScoreCommand(armPosition);
				// If we are moving to a forwards position, we need to move to an intermediate
				// position first
			} else {
				return new SequentialCommandGroup(new ArmScoreCommand(ArmPosition.TO_FORWARD_INTERMEDIATE),
						new ArmScoreCommand(armPosition));
			}
		}
	}
	// Drive out of community
	// https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.p
	public static Command getOutOfCommunityAuto(double driveDistance) { // start as close to line as possible, just
																		// drive forward
		return new DriveDistanceCommand(driveDistance); // about 0.8 probably
	}

	// Balance only
	// https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa5c6e9265_0_2
	public static Command getOnToChargerAuto(double driveDistance) { // start backed up a meter, figure the running
																		// start will be helpful
		return new SequentialCommandGroup(
				new DriveDistanceCommand(driveDistance),
				new BalancePIDCommand());
	}

	// Just Score High Game Piece
	// https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa5c6e9265_0_9
	public static Command getScorePieceAuto() { // start backed up a meter, figure the running start will be helpful
		return getPlacePieceCommand(null); // TODO: position
	}

	// Leave Community and balance
	// https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa8ee801ec_1_0
	public static Command getLeaveThenBalanceAuto(double driveDistance) { // Start right to the right of Charge station
		return new SequentialCommandGroup(
				new DriveDistanceCommand(driveDistance),
				new TurnCommand(-90),
				new DriveDistanceCommand(2), // maybe second parameter for how far this is?
				new TurnCommand(-90), // TODO: verify all these distances
				new DriveDistanceCommand(driveDistance + 0.5), // 0.5 should get on chargerstation, idk
				new BalancePIDCommand());
	}

	public static Command getScoreThenLeaveCommand(){
		return new SequentialCommandGroup(
			getEnsurePreloadCommand(),
			new ArmScoreCommand(ArmPosition.TO_BACK_INTERMEDIATE),
			new ArmScoreCommand(ArmPosition.HIGH_BACK),
			getOuttakePieceCommand(),
			new ArmScoreCommand(ArmPosition.TO_FORWARD_INTERMEDIATE),
			new ArmScoreCommand(ArmPosition.POCKET),
			new DriveDistanceCommand(4)

		);

	}
	public static Command getJustLeaveCommand(){
		return new SequentialCommandGroup(
			new DriveDistanceCommand(4)

		);

	}
	public static Command getOverTheFulcrumNoScoreAuto(){
		return new SequentialCommandGroup(
			new DriveDistanceCommand(-0.2),
			new DriveTimeCommand(-0.6, 500),
			new DriveTimeCommand(-0.25, 2500),
			new DriveTimeCommand(0.6, 750),
			new BalancePIDCommand()

		);

	}
	public static Command getLeaveThenScoreCommand() { // Start right to the right of Charge station
		return new SequentialCommandGroup(
			new SequentialCommandGroup(
				new ArmScoreCommand(ArmPosition.TO_BACK_INTERMEDIATE),
				new ArmScoreCommand(ArmPosition.HIGH_BACK).withTimeout(1.5)
			),
			getOuttakePieceCommand(),
			new ArmScoreCommand(ArmPosition.TO_FORWARD_INTERMEDIATE).withTimeout(1.5),
				new ParallelCommandGroup(
				//getEnsurePreloadCommand(),
				new TurnRelativeCommand(-0.75)),
				new ParallelRaceGroup(
					new DriveDistanceCommand(5.2),
					getPickupPieceCommand()
				), 
				new ParallelCommandGroup(
					new WheelGripperCommand(WheelGripperPosition.INTAKE_CUBE_W_SENSOR).withTimeout(1),
					new DriveTimeCommand(-0.25, 250)
				),
				new ParallelCommandGroup(
					new SequentialCommandGroup(
						new TurnRelativeCommand(0.5),
						new DriveDistanceCommand(-3.5),
						getAnvitaAuto(),
						new DriveTimeCommand(-0.15,1000)
					),
					new SequentialCommandGroup(
						new ArmScoreCommand(ArmPosition.TO_BACK_INTERMEDIATE),
						new ArmScoreCommand(ArmPosition.MEDIUM_BACK).withTimeout(1.5)
					)
				),
				getOuttakePieceCommand()
		/* new BalancePIDCommand() */);
	}

	// Score Preloaded and Engage
	// https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa94f33ee4_0_0
	public static Command getScoreThenBalanceAuto() { // Start lined up on center of Charge Station pushed up against
														// nodes
		return new SequentialCommandGroup(
				getEnsurePreloadCommand(),
				new ParallelCommandGroup(
						new ArmScoreCommand(ArmPosition.HIGH),
						new SequentialCommandGroup(
								new WaitCommand(1),
								new DriveTimeCommand(.25, 1250))),
				new ParallelCommandGroup(
						new SequentialCommandGroup(
								getOuttakePieceCommand(),
								new ArmScoreCommand(ArmPosition.LOW)),
						new SequentialCommandGroup(
								new DriveDistanceCommand(-0.2),
								new DriveTimeCommand(-0.7, 500))),
				new BalancePIDCommand());
	}
	public static Command getScoreOTBThenBalanceAuto() { // Start lined up on center of Charge Station pushed up against
														// nodes
		return new SequentialCommandGroup(
				getEnsurePreloadCommand(),
				new SequentialCommandGroup(
					new ArmScoreCommand(ArmPosition.TO_BACK_INTERMEDIATE),
					new ArmScoreCommand(ArmPosition.MEDIUM_BACK)
				),
				new SequentialCommandGroup(
								getOuttakePieceCommand(),
								new ArmScoreCommand(ArmPosition.TO_FORWARD_INTERMEDIATE),
								new ArmScoreCommand(ArmPosition.POCKET)),
						
						new SequentialCommandGroup(
								new DriveDistanceCommand(0.2),
								new DriveTimeCommand(0.6, 500),
								new DriveTimeCommand(0.25, 2300),
								new DriveTimeCommand(-0.6, 500),
								new BalancePIDCommand()
						)
		);
	}
	// Score, Over Charging Station -> Out of community, backup to balance
	// https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa8ee801ec_1_8
	public static Command getOverTheFulcrumAuto() { // start lined up with middle node of coopertition zone
		return new SequentialCommandGroup(
			getEnsurePreloadCommand(),
			new ParallelCommandGroup(
					new ArmScoreCommand(ArmPosition.HIGH),
					new SequentialCommandGroup(
							new WaitCommand(1),
							new DriveTimeCommand(.25, 1250))),
				new ParallelCommandGroup(
					new SequentialCommandGroup(
						getOuttakePieceCommand(),
						new ArmScoreCommand(ArmPosition.POCKET)
					),
					new SequentialCommandGroup(
						new DriveDistanceCommand(-0.2),
						new DriveTimeCommand(-0.6, 500),
						new DriveTimeCommand(-0.25, 2500),
						new DriveTimeCommand(0.6, 750),
						new BalancePIDCommand()

					)
				)//,
				// new BalancePIDCommand()
		);
	}
	// Score, Over Charging Station -> Out of community, backup to balance
	// https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1fa8ee801ec_1_8
	public static Command getOverTheFulcrumScoreAuto() { // start lined up with middle node of coopertition zone
		return new SequentialCommandGroup(
				getEnsurePreloadCommand(),
				new ParallelCommandGroup(
					new SequentialCommandGroup(
						new ArmScoreCommand(ArmPosition.TO_BACK_INTERMEDIATE),
						new ArmScoreCommand(ArmPosition.HIGH_BACK)
					),
					new SequentialCommandGroup(
						new WaitCommand(0.65),
						new DriveDistanceCommand(-0.6)
					)
				),
				new ParallelCommandGroup(
					new SequentialCommandGroup(
						getOuttakePieceCommand(),
						new ArmScoreCommand(ArmPosition.POCKET)
					),
					new SequentialCommandGroup(
						new DriveDistanceCommand(0.2),
						new DriveTimeCommand(0.7, 500),
						new DriveTimeCommand(0.25, 2300)//,
						// new DriveTimeCommand(0.7, 500),
						// new BalancePIDCommand()

					)
				)//,
				// new BalancePIDCommand()
		);
	}

	// Score, Leave Community, Intake, and then Score again
	// Also can be used to Score, Over Charging Station -> Out of Community, and
	// Score Again
	// https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g1b63d19ef3d_3_0
	public static Command getTwoScoreAuto() {
		return new SequentialCommandGroup(
				getPlacePieceCommand(null), // TODO: position
				new TurnCommand(-180),
				new DriveDistanceCommand(7),
				getPickupPieceCommand(),
				new TurnCommand(-180),
				new TagAlignCommand(),
				getPlacePieceCommand(null)); // TODO: position
	}

	// Score, Leave Community, Intake, Score again, and balance
	// https://docs.google.com/presentation/d/1O_zm6wuVwKJRE06Lj-Mtahat5X3m4VljtLzz4SqzGo4/edit#slide=id.g12845fa040c_0_5
	public static Command getTwoScoreBalanceAuto() {
		return new SequentialCommandGroup(
				getPlacePieceCommand(null), // TODO: need position
				new TurnCommand(-180),
				new DriveDistanceCommand(7),
				getPickupPieceCommand(),
				new TurnCommand(-180),
				new TagAlignCommand(),
				getPlacePieceCommand(null), // TODO: need position
				new TurnCommand(-90),
				new DriveDistanceCommand(2),
				new TurnCommand(-90),
				new DriveDistanceCommand(2), // Distances are guesses, need to adjust based on practice field
				new BalancePIDCommand());
	}

	// Pick up game piece
	public static Command getPickupPieceCommand() {
		return new SequentialCommandGroup(
				new SequentialCommandGroup(
						new ArmScoreCommand(ArmPosition.POCKET_INTERMEDIATE),
						new ArmScoreCommand(ArmPosition.LOW),
						new ArmScoreCommand(ArmPosition.HOLD),
						new WheelGripperCommand(WheelGripperPosition.INTAKE_CUBE_W_SENSOR)));
	}

	// Pick up game piece
	public static Command getEnsurePreloadCommand() {
		return new WheelGripperCommand(WheelGripperPosition.INTAKE_CUBE_W_SENSOR).withTimeout(0.5); // TODO fix
	}

	// Place game piece taking in position
	public static Command getPlacePieceCommand(ArmPosition position) {
		return new SequentialCommandGroup(
				new ArmScoreCommand(position),
				new WheelGripperCommand(WheelGripperPosition.OUTTAKE),
				new WaitCommand(0.5),
				new WheelGripperCommand(WheelGripperPosition.STOP));
	}

	// Place game piece
	public static Command getOuttakePieceCommand() {
		return new SequentialCommandGroup(
				new WheelGripperCommand(WheelGripperPosition.OUTTAKE),
				new WaitCommand(0.5),
				new WheelGripperCommand(WheelGripperPosition.STOP));
	}

	public static Command getALittleCloser() {
		double limelightz = Math.abs(AprilTagSubsystem.get().m_z);
		double limelightYaw = Math.toRadians(-AprilTagSubsystem.get().m_yaw);
		double limelightx = AprilTagSubsystem.get().m_x;

		double targetz = limelightz / 2;
		double distanceToTarget = Math.sqrt(Math.pow(limelightx, 2) + Math.pow(targetz, 2));
		double angleToTarget = Math.atan2(targetz, limelightx);
		double turnAngle1 = Math.PI / 2 - (angleToTarget + limelightYaw);
		double turnAngle2 = -1 * (turnAngle1 + limelightYaw);
		SmartDashboard.putNumber("distanceToTarget", distanceToTarget);
		SmartDashboard.putNumber("angleToTarget", Math.toDegrees(angleToTarget));
		SmartDashboard.putNumber("turnAngle1", Math.toDegrees(turnAngle1));
		SmartDashboard.putNumber("turnAngle2", Math.toDegrees(turnAngle2));

		return new SequentialCommandGroup(
				// new DriveDistanceCommand(-1.5).withTimeout(3)
				new TurnRelativeCommand(Math.toDegrees(turnAngle1)),
				new DriveDistanceCommand(-distanceToTarget).withTimeout(3),
				new TurnRelativeCommand(Math.toDegrees(turnAngle2)));

	}

	public static Command getAnvitaAuto() {
		return new DeferredCommandAuto(() -> getALittleCloser());
	}
}
