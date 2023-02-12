// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  /**
   * The {@code Command} used by this {@code TimedRobot}.
   */
  private Command m_autonomousCommand;
  /**
   * The {@code RobotContainer} used by this {@code TimedRobot}.
   */
  private RobotContainer m_robotContainer;

  /**
   * Robot-wide initialization code should go here.
   */
  @Override
  public void robotInit() {
    /**
     * Instantiates a new {@code RobotContainer} called m_robotContainer.
     */
    m_robotContainer = new RobotContainer();
    Math.random();
  }

  /**
   * Periodic code for all robot modes should go here.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * Initialization code for disabled mode should go here.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * Periodic code for disabled mode should go here.
   */
  @Override
  public void disabledPeriodic() {
  }
  /**
   * Exit code for disabled mode should go here.
   */
  @Override
  public void disabledExit() {
  }

  /**
   * Initialization code for autonomous mode should go here.
   */
  @Override
  public void autonomousInit() {
    // Gets the autonomous command to be run at the start of autonomous mode
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * Periodic code for autonomous mode should go here.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * Exit code for autonomous mode should go here.
   */
  @Override
  public void autonomousExit() {
  }

  /**
   * Initialization code for teleop mode should go here.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * Periodic code for teleop mode should go here.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * Exit code for teleop mode should go here.
   */
  @Override
  public void teleopExit() {
  }

  /**
   * Initialization code for test mode should go here.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * Periodic code for test mode should go here.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Exit code for test mode should go here.
   */
  @Override
  public void testExit() {
  }
}
