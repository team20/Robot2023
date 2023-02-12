// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {
	private static GripperSubsystem s_subsystem;

	private CANSparkMax m_gripperWinch = new CANSparkMax(GripperConstants.kWinchPort, MotorType.kBrushless);
    private final RelativeEncoder m_gripperWinchEncoder = m_gripperWinch.getEncoder();
	private SparkMaxLimitSwitch m_openlimitSwitch;
	private SparkMaxLimitSwitch m_closelimitSwitch;
    private final SparkMaxPIDController m_gripperWinchController = m_gripperWinch.getPIDController();

	/** Creates a new GripperSubsystem. */
	public GripperSubsystem() {
		final GenericHID m_controller = new GenericHID(frc.robot.Constants.ControllerConstants.kDriverControllerPort);
		m_openlimitSwitch = m_gripperWinch.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
		m_closelimitSwitch = m_gripperWinch.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);


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
		// m_gripperWinchEncoder.setZeroOffset(GripperConstants.kWinchEncoderZeroOffset);

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

	public void setGripperEncoderPosition(double position){
		m_gripperWinchController.setReference(position, ControlType.kPosition);
	}

	public static GripperSubsystem get() {
		return s_subsystem;
	}

	public boolean getOpenLimitSwitch(){
		return m_openlimitSwitch.isPressed();
	}

	public boolean getCloseLimitSwitch(){
		return m_closelimitSwitch.isPressed();
	}
}
