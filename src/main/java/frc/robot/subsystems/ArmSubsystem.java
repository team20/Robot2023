// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.ForwardKinematicsTool;
import frc.robot.util.InverseKinematicsTool;

public class ArmSubsystem extends SubsystemBase {
	/** Stores the instance of the ArmSubsystem */
	private static ArmSubsystem s_subsystem;
	private final CANSparkMax m_lowerArmMotor = new CANSparkMax(3, MotorType.kBrushless);
	// private final CANSparkMax m_lowerArmMotor2 = new
	// CANSparkMax(ArmConstants.kLowerMotor2ID, MotorType.kBrushless);
	private final CANSparkMax m_upperArmMotor = new CANSparkMax(4, MotorType.kBrushless);

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
	private boolean m_temporarilyDisabled;
	private boolean m_manualArmRan;

	private MedianFilter m_upperMotorMedianFilter = new MedianFilter(1000);
	private LinearFilter m_upperMotorAverageFilter = LinearFilter.movingAverage(1000);
	private double m_averageCurrentUpper;
	private double m_medianCurrentUpper;
	private double m_maxCurrent;

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
		m_lowerArmMotor.setInverted(ArmConstants.kLowerInvert);
		m_lowerArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_lowerArmMotor.enableVoltageCompensation(12);
		m_lowerArmMotor.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);
		SparkMaxLimitSwitch forwardSwitch = m_lowerArmMotor
				.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
		forwardSwitch.enableLimitSwitch(false);
		SparkMaxLimitSwitch reverseSwitch = m_lowerArmMotor
				.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
		reverseSwitch.enableLimitSwitch(false);

		m_lowerArmEncoder.setPositionConversionFactor(360);
		m_lowerArmEncoder.setZeroOffset(ArmConstants.kLowerEncoderZeroOffset);
		m_lowerArmEncoder.setInverted(true);

		m_lowerArmController.setP(ArmConstants.kLowerArmP);
		m_lowerArmController.setI(ArmConstants.kLowerArmI);
		m_lowerArmController.setIZone(ArmConstants.kLowerArmIz);
		m_lowerArmController.setD(ArmConstants.kLowerArmD);
		m_lowerArmController.setFF(ArmConstants.kLowerArmFF);
		m_lowerArmController.setOutputRange(ArmConstants.kMinOutputLower, ArmConstants.kMaxOutputLower);
		m_lowerArmController.setFeedbackDevice(m_lowerArmEncoder);
		// The lower arm doesn't need PID wrapping, it has a very specific range it
		// moves in
		m_lowerArmController.setPositionPIDWrappingEnabled(false);

		// Initialize 2nd lower arm motor
		// m_lowerArmMotor2.restoreFactoryDefaults();
		// m_lowerArmMotor2.setInverted(ArmConstants.kLowerInvert);
		// m_lowerArmMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
		// m_lowerArmMotor2.enableVoltageCompensation(12);
		// m_lowerArmMotor2.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);

		// SparkMaxLimitSwitch forwardSwitch2 = m_lowerArmMotor2
		// .getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
		// forwardSwitch2.enableLimitSwitch(false);
		// SparkMaxLimitSwitch reverseSwitch2 = m_lowerArmMotor2
		// .getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
		// reverseSwitch2.enableLimitSwitch(false);

		// // Make the 2nd lower arm motor follow the first one
		// // They point in opposite directions, so the 2nd motor needs to be inverted
		// m_lowerArmMotor2.follow(m_lowerArmMotor, ArmConstants.kLowerArmMotor2Oppose);

		// Initialize upper arm
		m_upperArmMotor.restoreFactoryDefaults();
		m_upperArmMotor.setInverted(ArmConstants.kUpperInvert);
		m_upperArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_upperArmMotor.enableVoltageCompensation(12);
		m_upperArmMotor.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);
		m_upperArmEncoder.setPositionConversionFactor(360);
		m_upperArmEncoder.setZeroOffset(ArmConstants.kUpperEncoderZeroOffset);
		m_upperArmEncoder.setInverted(true);

		m_upperArmController.setP(ArmConstants.kUpperArmP);
		m_upperArmController.setI(ArmConstants.kUpperArmI);
		m_upperArmController.setIZone(ArmConstants.kUpperArmIz);
		m_upperArmController.setD(ArmConstants.kUpperArmD);
		m_upperArmController.setFF(ArmConstants.kUpperArmFF);
		m_upperArmController.setOutputRange(ArmConstants.kMinOutputUpper, ArmConstants.kMaxOutputUpper);
		m_upperArmController.setFeedbackDevice(m_upperArmEncoder);
		// The upper arm can't spin clockwise without hitting the robot, so PID wrapping
		// is disabled
		m_upperArmController.setPositionPIDWrappingEnabled(false);

		m_targetLowerArmAngle = getLowerArmAngle();
		m_targetUpperArmAngle = getUpperArmAngle();
	}

	public static ArmSubsystem get() {
		return s_subsystem;
	}

	/**
	 * Sets the percent output for the lower arm motor
	 * 
	 * @param speed The percent output to run the motor at
	 */
	public void setLowerArmMotorSpeed(double speed) {
		m_manualArmRan = true;
		m_lowerArmMotor.set(speed);
	}

	public boolean getManualArmRan() {
		return m_manualArmRan;
	}

	/**
	 * Sets the percent output for the upper arm motor
	 * 
	 * @param speed The percent output to run the motor at
	 */
	public void setUpperArmMotorSpeed(double speed) {
		m_manualArmRan = true;
		m_upperArmMotor.set(speed);
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
	 * Sets and logs the angles the lower and upper arm will move to
	 * 
	 * @param lower The target angle for the lower arm
	 * @param upper The target angle for the upper arm
	 */
	public void setAngles(double lower, double upper) {
		SmartDashboard.putNumber("Target Lower Arm Angle", lower);
		SmartDashboard.putNumber("Target Upper Arm Angle", upper);
		// Prevent the lower arm from going more than 10 degrees behind vertical or
		// below 45 degrees
		// Prevent the upper arm from going more than 270 degrees or less than 15
		// degrees relative to the lower arm
		if ((lower <= ArmConstants.kLowerArmMaxAngle && lower >= ArmConstants.kLowerArmMinAngle)
				&& (upper <= ArmConstants.kUpperArmMaxAngle && upper >= ArmConstants.kUpperArmMinAngle)) {
			m_manualArmRan = false;
			m_targetLowerArmAngle = lower;
			m_lowerArmController.setReference(lower, ControlType.kPosition);

			m_targetUpperArmAngle = upper;
			m_upperArmController.setReference(upper, ControlType.kPosition);
		}
	}

	/**
	 * @return Whether or not the arm is at the angles we want it to be at
	 */
	public boolean isNearTargetAngle() {
		return checkAngle(m_targetLowerArmAngle, getLowerArmAngle()) &&
				checkAngle(m_targetUpperArmAngle, getUpperArmAngle());
	}

	public boolean isNearTargetAngleIntermediate() {
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
	public boolean checkAngle(double targetAngle, double currentAngle) {
		double upperAngleBound = targetAngle + ArmConstants.kAllowedDegreesError;
		double lowerAngleBound = targetAngle - ArmConstants.kAllowedDegreesError;
		// Simple bounds checking without accounting for wraparound
		if (Math.abs(currentAngle - targetAngle) < ArmConstants.kAllowedDegreesError) {
			return true;
			/*
			 * If there's wraparound, there's two parts of the accepted range: The part that
			 * hasn't wrapped around, and the part that has. e.g., targetAngle = 1,
			 * currentAngle = 358:
			 * The wrapped range is 357-5 degrees. wrappedLowerAngleBound covers 357-360,
			 * wrappedUpperAngleBound covers 0-5, so all angles are covered.
			 */
		} else if (lowerAngleBound < 0 || upperAngleBound > 360) {
			// System.out.println(0/0);
			double wrappedLowerAngleBound = MathUtil.inputModulus(lowerAngleBound, 0, 360);
			double wrappedUpperAngleBound = MathUtil.inputModulus(upperAngleBound, 0, 360);
			if (currentAngle > wrappedLowerAngleBound || currentAngle < wrappedUpperAngleBound) {
				return true;
			}
		}
		return false;
	}

	/* This is checkAngle but with a higher degree of error */
	public boolean checkAngleIntermediate(double targetAngle, double currentAngle) {
		double upperAngleBound = targetAngle + ArmConstants.kAllowedDegreesErrorIntermediate;
		double lowerAngleBound = targetAngle - ArmConstants.kAllowedDegreesErrorIntermediate;
		// Simple bounds checking without accounting for wraparound
		if (Math.abs(currentAngle - targetAngle) < ArmConstants.kAllowedDegreesErrorIntermediate) {
			return true;
			/*
			 * If there's wraparound, there's two parts of the accepted range: The part that
			 * hasn't wrapped around, and the part that has. e.g., targetAngle = 1,
			 * currentAngle = 358:
			 * The wrapped range is 357-5 degrees. wrappedLowerAngleBound covers 357-360,
			 * wrappedUpperAngleBound covers 0-5, so all angles are covered.
			 */
		} else if (lowerAngleBound < 0 || upperAngleBound > 360) {
			// System.out.println(0/0);
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
		if (m_upperArmMotor.getAppliedOutput() > 0) {
			double current = m_upperArmMotor.getOutputCurrent();
			m_medianCurrentUpper = m_upperMotorMedianFilter.calculate(current);
			m_averageCurrentUpper = m_upperMotorAverageFilter.calculate(current);
			if (current > m_maxCurrent) {
				m_maxCurrent = current;
			}
		}
		// SmartDashboard.putNumber("Lower Arm Motor Output", m_lowerArmMotor.getOut());
		// SmartDashboard.putNumber("Upper Arm Motor Output",
		// m_upperArmMotor.getAppliedOutput());
		SmartDashboard.putNumber("Lower Arm Motor Output", m_lowerArmMotor.getOutputCurrent());
		// SmartDashboard.putNumber("Lower Arm Motor 2 Output",
		// m_lowerArmMotor2.getOutputCurrent());
		SmartDashboard.putNumber("Upper Arm Motor Output", m_upperArmMotor.getAppliedOutput());
		SmartDashboard.putNumber("Upper Arm Motor Median Current", m_medianCurrentUpper);
		SmartDashboard.putNumber("Upper Arm Motor Average Current", m_averageCurrentUpper);
		SmartDashboard.putNumber("Upper Arm Motor Max Current", m_maxCurrent);

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
		if (armPosition != null) {
			SmartDashboard.putNumber("IK Lower Arm Angle", armPosition[0]);
			SmartDashboard.putNumber("IK Upper Arm Angle", armPosition[1]);
		}

		// If the y-coordinate of the upper arm is about to exceed the height, stop the
		// motors by setting their target angles to their current angles
		if (coordinates[1] > ArmConstants.kMaxHeight - 1) {
			ArmSubsystem.get().setAngles(ArmSubsystem.get().getLowerArmAngle(), ArmSubsystem.get().getUpperArmAngle());
		}
		if (!m_temporarilyDisabled && (getLowerArmAngle() <= ArmConstants.kLowerArmInvalidLowerBound
				|| getLowerArmAngle() >= ArmConstants.kLowerArmInvalidUpperBound)) {
			// setLowerArmMotorSpeed(0);
			// setUpperArmMotorSpeed(0);
			m_temporarilyDisabled = true;
		} else if (m_temporarilyDisabled && getLowerArmAngle() > ArmConstants.kLowerArmInvalidLowerBound
				&& getLowerArmAngle() < ArmConstants.kLowerArmInvalidUpperBound) {
			// setAngles(m_targetLowerArmAngle, m_targetUpperArmAngle);
			m_temporarilyDisabled = false;
		}

		// if(isNearTargetAngle()){
		// setLowerArmMotorSpeed(0);
		// setUpperArmMotorSpeed(0);
		// }
	}
}
