package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
//TODO: import pixy subsystem

public class TurnPixyCommand extends CommandBase {
  /** Creates a new TurnCommand. */
  private int m_targetX;
  private double m_turnTolerance;
  private PIDController m_turnController = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);

	/** Creates a new TurnCommand. */
  public TurnPixyCommand(int targetX, double turnTolerance) {
    m_targetX = targetX;
    m_turnTolerance = turnTolerance;
    addRequirements(DriveSubsystem.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnController.enableContinuousInput(-180, 180);
    m_turnController.setSetpoint(m_targetX);
    m_turnController.setTolerance(m_turnTolerance);
    //m_turnController.setIntegratorRange(-0.05, 0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // call pixycam subsystemm to get X
    int x = m_targetX; //TODO: replace m_targetX with pixysubsystem.x();
    double turnSpeed = m_turnController.calculate(x);

    //if turn speed is less than 0.1 make it 0.1 in the right direction
    turnSpeed = Math.abs(turnSpeed) < 0.15 ? Math.signum(turnSpeed)*0.15 : turnSpeed;

    turnSpeed = MathUtil.clamp(turnSpeed, -0.5, 0.5);
    DriveSubsystem.get().tankDrive(-turnSpeed, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Turn Ended");
    DriveSubsystem.get().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turnController.atSetpoint();
  }
}