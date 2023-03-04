package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnProfiledCommand extends CommandBase {
  /** Creates a new TurnCommand. */
  private double m_targetAngle;

  /*
   * upperBoundP = the highest value of P allowable for the turn
   * lowerBoundP = the lowest value of P allowable for the turn
   * lowerBoundError = the error value that matches to the highest value of p(small angle)
   * 
   */
  private double m_upperBoundP = 0.08;
  private double m_lowerBoundP = 0.02;
  private double m_lowerBoundError = 10;
  private PIDController m_turnController = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);

	/** Creates a new TurnCommand. 
   * @param targetAngle The angle to turn to(absolute)
   * The way this works is that the P value of the pid is adjusted based on our initial
   * error state - the higher the initial error, the less responsive we want the pid to be
   * this is because it takes some response to get started, and at a low initial error state
   * if the p-value is too small, it will either take a long time to achieve the target angle
   * or never reach it at all
   * on the other hand, for large initial error states, a p-value that is responsive at small
   * initial error states is too responsive, and the system overshoots, and takes a long time
   * to reach the desired end angle
   * If we find a p-value that works for the largest turn error of 180, and one that works for a 
   * small angle, such as 10, then we can assume a linear function of p between the two
  */
  public TurnProfiledCommand(double targetAngle) {
    m_targetAngle = targetAngle;
    addRequirements(DriveSubsystem.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnController.setSetpoint(m_targetAngle);

    m_turnController.enableContinuousInput(-180, 180);
    m_turnController.setTolerance(DriveConstants.kTurnTolerance);
    double currError = m_turnController.getPositionError();
    double p = this.getProportionalValue(currError);
    m_turnController.setP(p);
    //m_turnController.setIntegratorRange(-0.05, 0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
		double currAngle = DriveSubsystem.get().getHeading();
    double turnSpeed = m_turnController.calculate(currAngle);

    //do not allow an output of less than 0.1 in either direction(+ or -)
    //as this will not continue moving the robot
    //plateau at 0.1(in the correct direction)
    turnSpeed = Math.abs(turnSpeed) < 0.1 ? Math.signum(turnSpeed)*0.1 : turnSpeed;

    DriveSubsystem.get().tankDrive(-turnSpeed, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.get().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turnController.atSetpoint();
  }

  public double getProportionalValue(double error){

    //if we are below our tested lower bound for turn error
    //just return the highest value for p
    if(error < m_lowerBoundError){
      return m_upperBoundP;
    }
    //otherwise, we figure out the slope and y-intercept for our linear p-function
    //we have our lowest p at 180 deg error
    //and our highest p at the error set by lower bound error
    double slope = (m_lowerBoundP - m_upperBoundP) / (180 - m_lowerBoundError);
    double yIntercept = m_upperBoundP - (slope*m_lowerBoundError);
    //return the p-value calculated from our function
    return slope*error+yIntercept;
  }
}