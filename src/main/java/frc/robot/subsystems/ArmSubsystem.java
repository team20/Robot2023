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

// import shuffleboard logging later //;

public class ArmSubsystem extends SubsystemBase {
	/** Creates a new ArmSubsystem. :] */
	private static ArmSubsystem s_subsystem;

	// -------------------adjust encoders/pidcontrollers for 1 and 2 (& possibly 3)
	private final CANSparkMax m_lowerArmMotor = new CANSparkMax(ArmConstants.kLowerMotor, MotorType.kBrushless);
	private final CANSparkMax m_upperArmMotor = new CANSparkMax(ArmConstants.kUpperMotor, MotorType.kBrushless);

	private final SparkMaxAbsoluteEncoder m_lowerArmEncoder = m_lowerArmMotor
			.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
	private final SparkMaxAbsoluteEncoder m_upperArmEncoder = m_upperArmMotor
			.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
	private final SparkMaxPIDController m_lowerArmController = m_lowerArmMotor.getPIDController();
	private final SparkMaxPIDController m_upperArmController = m_upperArmMotor.getPIDController();
	private double m_xOffset = 0;
	private double m_yOffset = 0;

	/**
	 * Initializes a new instance of the {@link ArmSubsystem} class.
	 */

	// so i'm assuming there needs to be twice of what's below: for both
	// m_lowerArmMotor
	// and m_motor2 (joints... 1&2)
	public ArmSubsystem() {
		// Singleton
		if (s_subsystem != null) {
			try {
				throw new Exception("Arm subsystem already initalized!");
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
		s_subsystem = this;

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

		m_upperArmMotor.restoreFactoryDefaults();
		m_upperArmMotor.setInverted(ArmConstants.kInvert);
		m_upperArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_upperArmMotor.enableVoltageCompensation(12);
		m_upperArmMotor.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);
		m_upperArmEncoder.setPositionConversionFactor(360);
		m_upperArmEncoder.setZeroOffset(ArmConstants.kUpperEncoderZeroOffset);
		// now for second motor?:
		// m_motor.setSecondaryCurrentLimit(ArmConstants.kPeakCurrentLimit,
		// ArmConstants.kPeakCurrentDurationMillis);

		// pidController 3:
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

	public double getXOffset() {
		return m_xOffset;
	}

	public double getYOffset() {
		return m_yOffset;
	}

	public void setXOffset(double xOffset) {
		m_xOffset = xOffset;
	}

	public void setYOffset(double yOffset) {
		m_yOffset = yOffset;
	}

	public void changeXOffset(double xOffset) {
		m_xOffset += xOffset;
	}

	public void changeYOffset(double yOffset) {
		m_yOffset += yOffset;
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

	public double getLowerArmPosition() {
		return m_lowerArmEncoder.getPosition();
	}

	public double getUpperArmPosition() {
		return m_upperArmEncoder.getPosition();
	}

	public void setLowerArmPosition(double angle) {
		m_lowerArmController.setReference(angle, ControlType.kPosition);
		SmartDashboard.putNumber("Target Lower Arm Angle", angle);
	}

	public void setUpperArmPosition(double angle) {
		m_upperArmController.setReference(angle, ControlType.kPosition);
		SmartDashboard.putNumber("Target Upper Arm Angle", angle);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Current Lower Arm Angle", getLowerArmPosition());
		SmartDashboard.putNumber("Current Upper Arm Angle", getUpperArmPosition());

		double[] coordinates = ForwardKinematicsTool.getArmPosition(getUpperArmPosition(), getLowerArmPosition());
		SmartDashboard.putNumber("Forward X", coordinates[0]);
		SmartDashboard.putNumber("Forward Y", coordinates[1]);

		Double[] armPosition = InverseKinematicsTool.getArmAngles(coordinates[0], coordinates[1]);
		SmartDashboard.putNumber("inverse angle lower", armPosition[0]);
		SmartDashboard.putNumber("inverse angle upper", armPosition[1]);
		// SmartDashboard.putNumber("Raw Upper Arm position", )
		// System.out.println(m_upperArmController.getP());
	}
}
