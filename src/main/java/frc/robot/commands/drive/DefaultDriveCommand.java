package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.ArduinoConstants.LEDColors;
// import frc.robot.Constants.ArduinoConstants.LEDModes;
// import frc.robot.commands.ArduinoCommands.UpdateLEDsCommand;
// import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

	private final Supplier<Double> m_speedStraight, m_speedLeft, m_speedRight;
	//private final ArduinoSubsystem m_arduinoSubsystem;

	/**
	 * Drive using speed inputs as a percentage output of the motor
	 * 
	 * @param driveSubsystem The subsystem to be used
	 * @param speedStraight  Supplier of straight speed
	 * @param speedLeft      Supplier of left speed
	 * @param speedRight     Supplier of right speed
	 */
	public DefaultDriveCommand(Supplier<Double> speedStraight, Supplier<Double> speedLeft, Supplier<Double> speedRight) {
		//m_arduinoSubsystem = arduinoSubsystem;
		m_speedStraight = speedStraight;
		m_speedLeft = speedLeft;
		m_speedRight = speedRight;
		addRequirements(DriveSubsystem.get());
	}

	/**
	 * Update the motor outputs
	 */
	public void execute() {
		double speedStraight = Math.abs(m_speedStraight.get()) > .1 ? m_speedStraight.get()
				: 0;

		double speedLeft = Math.abs(m_speedLeft.get()) > 0.05 ? m_speedLeft.get() : 0;
		double speedRight = Math.abs(m_speedRight.get()) > 0.05 ? m_speedRight.get()
				: 0;

		if (speedStraight != 0) {
		speedLeft *= DriveConstants.kTurningMultiplier;
		speedRight *= DriveConstants.kTurningMultiplier;
		}

		DriveSubsystem.get().arcadeDrive(speedStraight, speedLeft, speedRight);

	}
}