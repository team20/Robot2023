// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

// import shuffleboard logging later //;

public class ArmSubsystem extends SubsystemBase {
	/** Creates a new ArmSubsystem. :] */
	private static ArmSubsystem s_subsystem;

	// -------------------adjust encoders/pidcontrollers for 1 and 2 (& possibly 3)
	private final CANSparkMax m_lowerArmMotor = new CANSparkMax(ArmConstants.kMotorPort3, MotorType.kBrushless);
	private final CANSparkMax m_upperArmMotor = new CANSparkMax(ArmConstants.kMotorPort4, MotorType.kBrushless);
	private final CANSparkMax m_motor1 = new CANSparkMax(ArmConstants.kMotorPort1, MotorType.kBrushless);
	private final CANSparkMax m_motor2 = new CANSparkMax(ArmConstants.kMotorPort2, MotorType.kBrushless);

	private final CANCoder m_lowerArmEncoder = new CANCoder(6);
	private final SparkMaxAbsoluteEncoder m_upperArmEncoder = m_upperArmMotor
			.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
	private final SparkMaxAbsoluteEncoder m_encoder1 = m_motor1
			.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
	private final SparkMaxAbsoluteEncoder m_encoder2 = m_motor2
			.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
	private final PIDController m_lowerArmController = new PIDController(ArmConstants.kP, ArmConstants.kI,
			ArmConstants.kD, 0.02);
	private final SparkMaxPIDController m_upperArmController = m_upperArmMotor.getPIDController();

	private double m_setPositionLowerArm = 0;
	private double m_setPositionUpperArm = 0;

	public static ArmSubsystem get() {
		return s_subsystem;
	}
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

		m_upperArmMotor.restoreFactoryDefaults();
		m_upperArmMotor.setInverted(ArmConstants.kInvert);
		m_upperArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		m_upperArmMotor.enableVoltageCompensation(12);
		m_upperArmMotor.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);
		// now for second motor?:
		m_motor1.restoreFactoryDefaults();
		m_motor2.restoreFactoryDefaults();
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

		// m_lowerArmController.setFeedbackDevice(m_lowerArmEncoder);
		// m_lowerArmController.setPositionPIDWrappingEnabled(true);
		// m_lowerArmController.setPositionPIDWrappingMaxInput(360);
		// m_lowerArmController.setPositionPIDWrappingMinInput(0);
		m_lowerArmController.enableContinuousInput(0, 360);
	}

	public void setSpeedUpper(double speed) {
		m_upperArmMotor.set(speed);
	}

	public void setSpeedLower(double speed) {
		m_lowerArmMotor.set(speed);
	}

	public double getLowerArmPosition() {
		return m_lowerArmEncoder.getAbsolutePosition();
	}

	public double getUpperArmPosition() {
		return m_upperArmEncoder.getPosition();
	}

	public void setLowerArmPosition(double angle) {
		m_lowerArmController.setSetpoint(angle);
	}

	public void setUpperArmPosition(double angle) {
		m_upperArmController.setReference(angle, ControlType.kPosition);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Lower Arm position", getLowerArmPosition());
		SmartDashboard.putNumber("Upper Arm position", getUpperArmPosition());
		SmartDashboard.putNumber("Motor 1 Position", m_encoder1.getPosition());
		SmartDashboard.putNumber("Motor 2 Position", m_encoder2.getPosition());
		double PIDoutput = m_lowerArmController.calculate(getLowerArmPosition());
		SmartDashboard.putNumber("PID Output", PIDoutput);
		setSpeedLower(MathUtil.clamp(PIDoutput, -1, 1));
		// SmartDashboard.putNumber("Raw Upper Arm position", )
		// System.out.println(m_upperArmController.getP());
	}
}
