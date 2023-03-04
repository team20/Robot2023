// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class WheelGripperSubsystem extends SubsystemBase {
	private static WheelGripperSubsystem s_subsystem;

	private CANSparkMax m_gripperMotor = new CANSparkMax(GripperConstants.kGripperID, MotorType.kBrushless);
	private final RelativeEncoder m_gripperEncoder = m_gripperMotor.getEncoder();
	private SparkMaxLimitSwitch m_forwardLimitSwitch;

	/** Creates a new GripperSubsystem. */
	public WheelGripperSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Gripper subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;
		m_forwardLimitSwitch = m_gripperMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
		m_forwardLimitSwitch.enableLimitSwitch(false);
		m_gripperMotor.restoreFactoryDefaults();
		m_gripperMotor.setInverted(GripperConstants.kInvert);
		m_gripperMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_gripperMotor.enableVoltageCompensation(12);
		m_gripperMotor.setSmartCurrentLimit(GripperConstants.kSmartCurrentLimit);

	}

	public static WheelGripperSubsystem get() {
		return s_subsystem;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Gripper output", m_gripperMotor.getOutputCurrent());
	}

	public void setGripperMotor(double speed) {
		SmartDashboard.putNumber("target gripper speed", speed);
		m_gripperMotor.set(speed);
	}

	public boolean getLimitSwitch() {
		return m_forwardLimitSwitch.isPressed();
	}
}