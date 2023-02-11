// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.GripperConstants;

public class AbsoluteGripperSubsystem extends SubsystemBase {
	private static AbsoluteGripperSubsystem s_subsystem;

	private CANSparkMax m_gripperWinch = new CANSparkMax(GripperConstants.kWinchPort, MotorType.kBrushless);
    private final SparkMaxAbsoluteEncoder m_gripperWinchEncoder = m_gripperWinch
			.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private final SparkMaxPIDController m_gripperWinchController = m_gripperWinch.getPIDController();

	/** Creates a new GripperSubsystem. */
	public AbsoluteGripperSubsystem() {
		final GenericHID m_controller = new GenericHID(frc.robot.Constants.ControllerConstants.kDriverControllerPort);
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Gripper subsystem already initalized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;

        m_gripperWinch.restoreFactoryDefaults();
		m_gripperWinch.setInverted(GripperConstants.kInvert);
		m_gripperWinch.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_gripperWinch.enableVoltageCompensation(12);
		m_gripperWinch.setSmartCurrentLimit(GripperConstants.kSmartCurrentLimit);
		m_gripperWinchEncoder.setPositionConversionFactor(360);
		m_gripperWinchEncoder.setZeroOffset(GripperConstants.kWinchEncoderZeroOffset);

        m_gripperWinchController.setP(GripperConstants.kP);
		m_gripperWinchController.setI(GripperConstants.kI);
		m_gripperWinchController.setIZone(GripperConstants.kIz);
		m_gripperWinchController.setD(GripperConstants.kD);
		m_gripperWinchController.setOutputRange(GripperConstants.kMinOutput, GripperConstants.kMaxOutput);
		m_gripperWinchController.setFeedbackDevice(m_gripperWinchEncoder);
		m_gripperWinchController.setPositionPIDWrappingEnabled(true);
		m_gripperWinchController.setPositionPIDWrappingMaxInput(360);
		m_gripperWinchController.setPositionPIDWrappingMinInput(0);
	}

	@Override
	public void periodic() {
	}

	public void setGripperMotor(double speed) {
		m_gripperWinch.set(speed);
	}

	public double getGripperEncoderPosition(){
		return m_gripperWinchEncoder.getPosition();
	}

	public static AbsoluteGripperSubsystem get() {
		return s_subsystem;
	}
}
