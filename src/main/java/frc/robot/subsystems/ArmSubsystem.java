// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
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
	/** Stores the angle we want the lower arm to be at */
	private double m_targetLowerArmAngle = 0;
	/** Stores the angle we want the upper arm to be at */
	private double m_targetUpperArmAngle = 0;

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
		// The lower arm doesn't need PID wrapping, it has a very specific range it
		// moves in
		m_lowerArmController.setPositionPIDWrappingEnabled(false);

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
		// The upper arm can't spin clockwise without hitting the robot, so PID wrapping
		// is disabled
		m_upperArmController.setPositionPIDWrappingEnabled(false);
	}

	public static ArmSubsystem get() {
		return s_subsystem;
	}

	/**
	 * @return The angle of the lower arm in degrees
	 */
	public double getLowerArmAngle() {
		return m_lowerArmEncoder.getPosition();
	}

	/**
	 * @return The angle of the upper arm in degrees
	 */
	public double getUpperArmAngle() {
		return m_upperArmEncoder.getPosition();
	}

	/**
	 * Sets the angle for the lower arm, and logs the angle
	 * 
	 * @param angle The target angle of the lower arm in degrees
	 */
	public void setLowerArmAngle(double angle) {
		setAngles(angle, m_targetUpperArmAngle);
	}

	/**
	 * Sets the angle for the upper arm, and logs the angle
	 * 
	 * @param angle The target angle of the upper arm in degrees
	 */
	public void setUpperArmAngle(double angle) {
		setAngles(m_targetLowerArmAngle, angle);
	}

	public void setAngles(double lower, double upper) {
		m_targetLowerArmAngle = lower;
		m_lowerArmController.setReference(lower, ControlType.kPosition);
		SmartDashboard.putNumber("Target Lower Arm Angle", lower);

		m_targetUpperArmAngle = upper;
		m_upperArmController.setReference(upper, ControlType.kPosition);
		SmartDashboard.putNumber("Target Upper Arm Angle", upper);
	}

	/**
	 * @return Whether or not the arm is at the angles we want it to be at
	 */
	public boolean isNearTargetAngle() {
		return checkAngle(m_targetLowerArmAngle, getLowerArmAngle()) &&
				checkAngle(m_targetUpperArmAngle, getUpperArmAngle());
	}

	/**
	 * Takes a target angle and current angle, and checks if the current angle is
	 * close enough to the target angle. The number of degrees the current angle can
	 * be different from the target angle is defined
	 * by {@link ArmConstants.kAllowedDegreesError}
	 * 
	 * @param targetAngle  The target angle
	 * @param currentAngle The current angle
	 * @return Whether or not the current angle is close enough to the target angle
	 */
	private boolean checkAngle(double targetAngle, double currentAngle) {
		double upperAngleBound = targetAngle + ArmConstants.kAllowedDegreesError;
		double lowerAngleBound = targetAngle - ArmConstants.kAllowedDegreesError;
		// Simple bounds checking without accounting for wraparound
		if (currentAngle < upperAngleBound && currentAngle > lowerAngleBound) {
			return true;
			/*
			 * If there's wraparound, there's two parts of the accepted range: The part that
			 * hasn't wrapped around, and the part that has. e.g., targetAngle = 1,
			 * currentAngle = 358:
			 * The wrapped range is 357-5 degrees. wrappedLowerAngleBound covers 357-360,
			 * wrappedUpperAngleBound covers 0-5, so all angles are covered.
			 */
		} else if (lowerAngleBound < 0 || upperAngleBound > 360) {
			double wrappedLowerAngleBound = MathUtil.inputModulus(lowerAngleBound, 0, 360);
			double wrappedUpperAngleBound = MathUtil.inputModulus(upperAngleBound, 0, 360);
			if (currentAngle > wrappedLowerAngleBound || currentAngle < wrappedUpperAngleBound) {
				return true;
			}
		}
		return false;
	}

	// This method will be called once per scheduler run
	@Override
	public void periodic() {
		SmartDashboard.putNumber("Lower Arm Motor Output", m_lowerArmMotor.getAppliedOutput());
		SmartDashboard.putNumber("Upper Arm Motor Output", m_upperArmMotor.getAppliedOutput());
		// Log the lower and upper arm angle as measured by the encoders
		SmartDashboard.putNumber("Current Lower Arm Angle", getLowerArmAngle());
		SmartDashboard.putNumber("Current Upper Arm Angle", getUpperArmAngle());
		// Calculate the arm position using the encoder angles
		double[] coordinates = ForwardKinematicsTool.getArmPosition(getLowerArmAngle(), getUpperArmAngle());
		SmartDashboard.putNumber("Forward X", coordinates[0]);
		SmartDashboard.putNumber("Forward Y", coordinates[1]);
		// Take the calculated arm position from the forward kinematics code, and
		// calculate the lower and upper arm angles to make sure everything works
		double[] armPosition = InverseKinematicsTool.calculateArmAngles(coordinates[0], coordinates[1]);
		SmartDashboard.putNumber("IK Lower Arm Angle", armPosition[0]);
		SmartDashboard.putNumber("IK Upper Arm Angle", armPosition[1]);
	}
}
