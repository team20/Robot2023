// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {
	private static GripperSubsystem s_subsystem;
	private TalonSRX m_gripperWinch = new TalonSRX(GripperConstants.kWinchPort);
	private CANCoder m_lowerCANCoder = new CANCoder(6);
	private CANCoder m_upperCANCoder = new CANCoder(7);
	private DigitalInput m_leftBumpSwitch = new DigitalInput(GripperConstants.kLeftBumpSwitchPort);
	private DigitalInput m_rightBumpSwitch = new DigitalInput(GripperConstants.kRightBumpSwitchPort);

	/** Creates a new GripperSubsystem. */
	public GripperSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Gripper subsystem already initalized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("Left Bump Switch", getLeftBumpSwitch());
		SmartDashboard.putBoolean("Right Bump Switch", getRightBumpSwitch());
	}

	public double getLowerJointAngle() {
		return m_lowerCANCoder.getAbsolutePosition();
	}

	public double getUpperJointAngle() {
		return m_upperCANCoder.getAbsolutePosition();
	}

	public void setGripperMotor(double speed) {
		m_gripperWinch.set(TalonSRXControlMode.PercentOutput, speed);
	}

	public boolean getLeftBumpSwitch() {
		return m_leftBumpSwitch.get();
	}

	public boolean getRightBumpSwitch() {
		return m_rightBumpSwitch.get();
	}

	public static GripperSubsystem get() {
		return s_subsystem;
	}
}
