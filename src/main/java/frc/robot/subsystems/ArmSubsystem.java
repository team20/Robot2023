// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

// import shuffleboard logging later //;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. :] */

  //--different: 2 motors not 1 :) idk if i did it right
  //-------------------adjust encoders/pidcontrollers for 1 and 2 
  private final CANSparkMax m_motor1 = new CANSparkMax(ArmConstants.kMotorPort1, MotorType.kBrushless);
  
  private final RelativeEncoder m_encoder1 = m_motor1.getEncoder();

  private final SparkMaxPIDController m_pidController1 = m_motor1.getPIDController();
  
  private double m_setPosition = 0;

  public enum Position {
    DOWN_POSITION,
    UP_POSITION
  }


  private final double downPositionEncoderPosition = 36.75; // TODO find encoder position
  private final double upPositionEncoderPosition = 0; // TODO find encoder position

  private static ArmSubsystem s_system;

  public static ArmSubsystem get() {
    return s_system;
  }
  /* 
  Initializes a new instance of the {@link ArmSubsystem} class.
  */



  // so i'm assuming there needs to be twice of what's below: for both m_motor1 and m_motor2 (joints... 1&2)
 public ArmSubsystem() {
    s_system = this;
    m_motor1.restoreFactoryDefaults();
    m_motor1.setInverted(ArmConstants.kInvert);
    m_motor1.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_motor1.enableVoltageCompensation(12);
    m_motor1.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);
    //now for second motor?:

    // m_motor.setSecondaryCurrentLimit(ArmConstants.kPeakCurrentLimit,  ArmConstants.kPeakCurrentDurationMillis);


    //pidController 1:
    m_pidController1.setP(ArmConstants.kP);
    m_pidController1.setI(ArmConstants.kI);
    m_pidController1.setIZone(ArmConstants.kIz);
    m_pidController1.setD(ArmConstants.kD);
    m_pidController1.setFF(ArmConstants.kFF);
    m_pidController1.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

    m_pidController1.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, ArmConstants.kSlotID);
    m_pidController1.setSmartMotionMaxAccel(ArmConstants.kMaxAcel, ArmConstants.kSlotID);
    m_pidController1.setSmartMotionMaxVelocity(ArmConstants.kMaxVelocity, ArmConstants.kSlotID);
    m_pidController1.setSmartMotionAllowedClosedLoopError(ArmConstants.kAllowedError, ArmConstants.kSlotID);
    m_pidController1.setSmartMotionMinOutputVelocity(ArmConstants.kMinVelocity, ArmConstants.kSlotID);

    resetm_encoder1();

  }

   private void resetm_encoder1() {
    m_encoder1.setPosition(0);
}



  public void setSpeed(double speed) {

    m_motor1.set(speed);
  }

  public double GetEncoderPosition() {
    return m_encoder1.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
