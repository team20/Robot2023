// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {
	private static GripperSubsystem s_subsystem;

	private CANSparkMax m_gripperScrew = new CANSparkMax(GripperConstants.kGripperID, MotorType.kBrushless);
	private final RelativeEncoder m_gripperScrewEncoder = m_gripperScrew.getEncoder();
	private SparkMaxLimitSwitch m_openLimitSwitch;

	/** Creates a new GripperSubsystem. */
	public GripperSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Gripper subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;
		m_openLimitSwitch = m_gripperScrew.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
		m_openLimitSwitch.enableLimitSwitch(getOpenLimitSwitch());
		m_gripperScrew.restoreFactoryDefaults();
		m_gripperScrew.setInverted(GripperConstants.kInvert);
		m_gripperScrew.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_gripperScrew.enableVoltageCompensation(12);
		m_gripperScrew.setSmartCurrentLimit(GripperConstants.kSmartCurrentLimit);
	}

	public static GripperSubsystem get() {
		return s_subsystem;
	}

	@Override
	public void periodic() {
	}

	public void setGripperMotor(double speed) {
		m_gripperScrew.set(speed);
	}

	public void resetZero() {
		m_gripperScrewEncoder.setPosition(0);
	}

	public double getGripperEncoderPosition() {
		return m_gripperScrewEncoder.getPosition();
	}

	public boolean getOpenLimitSwitch() {
		return m_openLimitSwitch.isPressed();
	}
}