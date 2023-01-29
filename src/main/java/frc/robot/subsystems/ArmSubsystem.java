// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

	private final RelativeEncoder m_lowerArmEncoder = m_lowerArmMotor.getEncoder();
	private final RelativeEncoder m_upperArmEncoder = m_upperArmMotor.getAlternateEncoder(8192);


	private final SparkMaxPIDController m_lowerArmController = m_lowerArmMotor.getPIDController();
	private final SparkMaxPIDController m_upperArmController = m_upperArmMotor.getPIDController();

	private double m_setPositionLowerArm = 0;
	private double m_setPositionUpperArm = 0;

	public static ArmSubsystem get() {
		return s_subsystem;
	}
	/*
	 * Initializes a new instance of the {@link ArmSubsystem} class.
	 */

	// so i'm assuming there needs to be twice of what's below: for both m_lowerArmMotor
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

		// m_motor.setSecondaryCurrentLimit(ArmConstants.kPeakCurrentLimit,
		// ArmConstants.kPeakCurrentDurationMillis);

		// pidController 3:
		m_upperArmController.setP(ArmConstants.kP);
		m_upperArmController.setI(ArmConstants.kI);
		m_upperArmController.setIZone(ArmConstants.kIz);
		m_upperArmController.setD(ArmConstants.kD);
		m_upperArmController.setFF(ArmConstants.kFF);
		m_upperArmController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
		// ---------------------
		m_lowerArmController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, ArmConstants.kSlotID);
		m_lowerArmController.setSmartMotionMaxAccel(ArmConstants.kMaxAcel, ArmConstants.kSlotID);
		m_lowerArmController.setSmartMotionMaxVelocity(ArmConstants.kMaxVelocity, ArmConstants.kSlotID);
		m_lowerArmController.setSmartMotionAllowedClosedLoopError(ArmConstants.kAllowedError, ArmConstants.kSlotID);
		m_lowerArmController.setSmartMotionMinOutputVelocity(ArmConstants.kMinVelocity, ArmConstants.kSlotID);

		resetEncoders();

	}

	private void resetEncoders() {
		m_lowerArmEncoder.setPosition(0);
		m_upperArmEncoder.setPosition(0);
	}

	public void setSpeed(double speed) {

		m_lowerArmMotor.set(speed);
		m_upperArmMotor.set(speed);
	}

	public double getLowerArmPosition() {		
		return (Math.abs(m_lowerArmEncoder.getPosition()*180) % 360);
	}

	public double getUpperArmPosition() {		
		return (Math.abs(m_upperArmEncoder.getPosition()*180) % 360);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Lower Arm position", ArmSubsystem.get().getLowerArmPosition());
		SmartDashboard.putNumber("Upper Arm position", ArmSubsystem.get().getUpperArmPosition());

	}
}
