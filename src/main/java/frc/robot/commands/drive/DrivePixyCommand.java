package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
//TODO: import pixy subsystem

public class DrivePixyCommand extends CommandBase {
  /** Creates a new TurnCommand. */
  private int m_targetWidth;
  private double m_widthTolerance;
  private PIDController m_controller = new PIDController(0.3, 0, 0);

	/** Creates a new TurnCommand. */
  public DrivePixyCommand(int targetWidth, double widthTolerance) {
    m_targetWidth = targetWidth;
    m_widthTolerance = widthTolerance;
    addRequirements(DriveSubsystem.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.enableContinuousInput(-180, 180);
    m_controller.setSetpoint(m_targetWidth);
    m_controller.setTolerance(m_widthTolerance);
    //m_turnController.setIntegratorRange(-0.05, 0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // call pixycam subsystemm to get X
    int width = m_targetWidth; //TODO: replace m_targetWidth with pixysubsystem.width();
    double speed = m_controller.calculate(width);

    //if turn speed is less than 0.1 make it 0.1 in the right direction
    speed = Math.abs(speed) < 0.15 ? Math.signum(speed)*0.15 : speed;

    speed = MathUtil.clamp(speed, -0.5, 0.5);
    DriveSubsystem.get().tankDrive(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.get().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atSetpoint();
  }
}