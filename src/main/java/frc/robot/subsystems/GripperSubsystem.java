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

	private CANSparkMax m_gripperScrew = new CANSparkMax(GripperConstants.kPort, MotorType.kBrushless);
    private final RelativeEncoder m_gripperScrewEncoder = m_gripperScrew.getEncoder();
	private SparkMaxLimitSwitch m_openlimitSwitch;
	private SparkMaxLimitSwitch m_closelimitSwitch;
    private final SparkMaxPIDController m_gripperScrewController = m_gripperScrew.getPIDController();

	/** Creates a new GripperSubsystem. */
	public GripperSubsystem() {
		m_openlimitSwitch = m_gripperScrew.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
		m_closelimitSwitch = m_gripperScrew.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);


		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Gripper subsystem already initalized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;

        m_gripperScrew.restoreFactoryDefaults();
		m_gripperScrew.setInverted(GripperConstants.kInvert);
		m_gripperScrew.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_gripperScrew.enableVoltageCompensation(12);
		m_gripperScrew.setSmartCurrentLimit(GripperConstants.kSmartCurrentLimit);
		m_gripperScrewEncoder.setPositionConversionFactor(360);

        m_gripperScrewController.setP(GripperConstants.kP);
		m_gripperScrewController.setI(GripperConstants.kI);
		m_gripperScrewController.setIZone(GripperConstants.kIz);
		m_gripperScrewController.setD(GripperConstants.kD);
		m_gripperScrewController.setOutputRange(GripperConstants.kMinOutput, GripperConstants.kMaxOutput);
		m_gripperScrewController.setFeedbackDevice(m_gripperScrewEncoder);
		m_gripperScrewController.setPositionPIDWrappingEnabled(true);
		m_gripperScrewController.setPositionPIDWrappingMaxInput(360);
		m_gripperScrewController.setPositionPIDWrappingMinInput(0);
	}

	@Override
	public void periodic() {
	}

	public void setGripperMotor(double speed) {
		m_gripperScrew.set(speed);
	}

	public double getGripperEncoderPosition(){
		return m_gripperScrewEncoder.getPosition();
	}

	public void setGripperEncoderPosition(double position){
		m_gripperScrewController.setReference(position, ControlType.kPosition);
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
