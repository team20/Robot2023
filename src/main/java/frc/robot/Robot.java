// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Method;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CommandComposer;

public class Robot extends TimedRobot {

	private Command m_autonomousCommand;
	// private Thread m_visionThread;
	private RobotContainer m_robotContainer;

	@Override
	public void robotInit() {
		for (Method method : CommandComposer.class.getMethods()) {
			for (Class clazz : method.getParameterTypes()) {
				System.out.print(clazz.getCanonicalName());
			}
			System.out.println();
		}
		SmartDashboard.putData(CommandScheduler.getInstance());
		m_robotContainer = new RobotContainer();

		// m_visionThread = new Thread(() -> {
		// Get the UsbCamera from CameraServer
		UsbCamera camera = CameraServer.startAutomaticCapture();
		// Set the resolution
		camera.setResolution(320, 240);
		// });
		// m_visionThread.setDaemon(true);
		// m_visionThread.start();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}
