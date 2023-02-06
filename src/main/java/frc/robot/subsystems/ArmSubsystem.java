// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.ForwardKinematicsTool;
import frc.robot.util.InverseKinematicsTool;

public class ArmSubsystem extends SubsystemBase {
	/** Stores the instance of the ArmSubsystem */
	private static ArmSubsystem s_subsystem;
	private final CANSparkMax m_lowerArmMotor = new CANSparkMax(ArmConstants.kLowerMotor, MotorType.kBrushless);
	private final CANSparkMax m_upperArmMotor = new CANSparkMax(ArmConstants.kUpperMotor, MotorType.kBrushless);

	private final SparkMaxAbsoluteEncoder m_lowerArmEncoder = m_lowerArmMotor
			.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
	private final SparkMaxAbsoluteEncoder m_upperArmEncoder = m_upperArmMotor
			.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

	private final SparkMaxPIDController m_lowerArmController = m_lowerArmMotor.getPIDController();
	private final SparkMaxPIDController m_upperArmController = m_upperArmMotor.getPIDController();

	/**
	 * Instantiate a new instance of the {@link ArmSubsystem} class.
	 */
	public ArmSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Arm subsystem already initialized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;
		// Initialize lower arm
		m_lowerArmMotor.restoreFactoryDefaults();
		m_lowerArmMotor.setInverted(ArmConstants.kInvert);
		m_lowerArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_lowerArmMotor.enableVoltageCompensation(12);
		m_lowerArmMotor.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);
		m_lowerArmEncoder.setPositionConversionFactor(360);
		m_lowerArmEncoder.setZeroOffset(ArmConstants.kLowerEncoderZeroOffset);

		m_lowerArmController.setP(ArmConstants.kP);
		m_lowerArmController.setI(ArmConstants.kI);
		m_lowerArmController.setIZone(ArmConstants.kIz);
		m_lowerArmController.setD(ArmConstants.kD);
		m_lowerArmController.setFF(ArmConstants.kFF);
		m_lowerArmController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
		m_lowerArmController.setFeedbackDevice(m_lowerArmEncoder);
		m_lowerArmController.setPositionPIDWrappingEnabled(true);
		m_lowerArmController.setPositionPIDWrappingMaxInput(360);
		m_lowerArmController.setPositionPIDWrappingMinInput(0);

		// Initialize upper arm
		m_upperArmMotor.restoreFactoryDefaults();
		m_upperArmMotor.setInverted(ArmConstants.kInvert);
		m_upperArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_upperArmMotor.enableVoltageCompensation(12);
		m_upperArmMotor.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);
		m_upperArmEncoder.setPositionConversionFactor(360);
		m_upperArmEncoder.setZeroOffset(ArmConstants.kUpperEncoderZeroOffset);

		m_upperArmController.setP(ArmConstants.kP);
		m_upperArmController.setI(ArmConstants.kI);
		m_upperArmController.setIZone(ArmConstants.kIz);
		m_upperArmController.setD(ArmConstants.kD);
		m_upperArmController.setFF(ArmConstants.kFF);
		m_upperArmController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
		m_upperArmController.setFeedbackDevice(m_upperArmEncoder);
		m_upperArmController.setPositionPIDWrappingEnabled(true);
		m_upperArmController.setPositionPIDWrappingMaxInput(360);
		m_upperArmController.setPositionPIDWrappingMinInput(0);

		// ---------------------
		// m_lowerArmController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,
		// ArmConstants.kSlotID);
		// m_lowerArmController.setSmartMotionMaxAccel(ArmConstants.kMaxAcel,
		// ArmConstants.kSlotID);
		// m_lowerArmController.setSmartMotionMaxVelocity(ArmConstants.kMaxVelocity,
		// ArmConstants.kSlotID);
		// m_lowerArmController.setSmartMotionAllowedClosedLoopError(ArmConstants.kAllowedError,
		// ArmConstants.kSlotID);
		// m_lowerArmController.setSmartMotionMinOutputVelocity(ArmConstants.kMinVelocity,
		// ArmConstants.kSlotID);
	}

	public static ArmSubsystem get() {
		return s_subsystem;
	}

	public void setSpeedUpper(double speed) {
		m_upperArmMotor.set(speed);
	}

	public void setSpeedLower(double speed) {
		m_lowerArmMotor.set(speed);
	}

	/**
	 * @return
	 *         The angle of the lower arm in degrees
	 */
	public double getLowerArmAngle() {
		return m_lowerArmEncoder.getPosition();
	}

	/**
	 * @return
	 *         The angle of the upper arm in degrees
	 */
	public double getUpperArmAngle() {
		return m_upperArmEncoder.getPosition();
	}

	/**
	 * Sets the angle for the lower arm, and logs the angle
	 * 
	 * @param angle
	 *              The target angle of the lower arm in degrees
	 */
	public void setLowerArmAngle(double angle) {
		m_lowerArmController.setReference(angle, ControlType.kPosition);
		SmartDashboard.putNumber("Target Lower Arm Angle", angle);
	}

	/**
	 * Sets the angle for the upper arm, and logs the angle
	 * 
	 * @param angle
	 *              The target angle of the upper arm in degrees
	 */
	public void setUpperArmAngle(double angle) {
		m_upperArmController.setReference(angle, ControlType.kPosition);
		SmartDashboard.putNumber("Target Upper Arm Angle", angle);
	}

	// This method will be called once per scheduler run
	@Override
	public void periodic() {
		// Log the lower and upper arm angle as measured by the encoders
		SmartDashboard.putNumber("Current Lower Arm Angle", getLowerArmAngle());
		SmartDashboard.putNumber("Current Upper Arm Angle", getUpperArmAngle());
		// Calculate the arm position using the encoder angles
		double[] coordinates = ForwardKinematicsTool.getArmPosition(getUpperArmAngle(), getLowerArmAngle());
		SmartDashboard.putNumber("Forward X", coordinates[0]);
		SmartDashboard.putNumber("Forward Y", coordinates[1]);
		// Take the calculated arm position from the forward kinematics code, and
		// calculate the lower and upper arm angles to make sure everything works
		Double[] armPosition = InverseKinematicsTool.getArmAngles(coordinates[0], coordinates[1]);
		SmartDashboard.putNumber("IK Lower Arm Angle", armPosition[0]);
		SmartDashboard.putNumber("IK Upper Arm Angle", armPosition[1]);
	}
}
