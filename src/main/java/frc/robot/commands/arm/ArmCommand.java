// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// 
package frc.robot.commands.arm;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  public enum Operation{
    CMD_ARM_UP,
    CMD_ARM_DOWN,
    CMD_ARM_SETTLE,
    CMD_RESET_ENCODER,
    CMD_ARM_MANUAL,
    CMD_ARM_STOP,
    CMD_ARM_MANUAL_DOWN
  }

  private Operation m_operation;
  public ArmCommand(Operation operation) {
    m_operation = operation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("scheduling arm");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
    if(m_operation == Operation.CMD_ARM_UP){
      ArmSubsystem.get().setPosition(ArmSubsystem.Position.UP_POSITION);
      ArmSubsystem.get().setBrakeMode();
      //System.out.println("setting to up");
      // ArmSubsystem.get().setPercentOutput(0);//TODO find speed
    } else if(m_operation == Operation.CMD_ARM_DOWN){
      //System.out.println("setting to down");
      ArmSubsystem.get().setPosition(ArmSubsystem.Position.DOWN_POSITION);
      ArmSubsystem.get().setCoastMode();
      // ArmSubsystem.get().setPercentOutput(0);//TODO find speed
    } 
    else if (m_operation==Operation.CMD_ARM_MANUAL_DOWN) {
      ArmSubsystem.get().setPercentOutput(.5);
    }
    else if(m_operation==Operation.CMD_RESET_ENCODER){
      ArmSubsystem.get().resetEncoder();
    }
    else if(m_operation==Operation.CMD_ARM_MANUAL){
      ArmSubsystem.get().setPercentOutput(-.6);
    }
    else if(m_operation==Operation.CMD_ARM_STOP){
      ArmSubsystem.get().setPercentOutput(0);
    }
*/

    ArmSubsystem.get().setSpeed(.2);
    
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {   
    ArmSubsystem.get().setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double encoderoutput1 = ArmSubsystem.get().GetEncoderPosition1();
    double encoderoutput2 = ArmSubsystem.get().GetEncoderPosition2();
    double encoderoutput3 = ArmSubsystem.get().GetEncoderPosition3();
    double encoderoutput4 = ArmSubsystem.get().GetEncoderPosition4();

    System.out.println("encoder1: " + encoderoutput1);
    System.out.println("encoder2: " + encoderoutput2);
    System.out.println("encoder3: " + encoderoutput3);
    System.out.println("encoder4: " + encoderoutput4);
    // double elapsed = Duration.between(m_startTime, Instant.now()).toMillis();    
    /* if(m_operation == Operation.CMD_ARM_UP || m_operation == Operation.CMD_ARM_DOWN){
      return true;
    }else if(m_operation == Operation.CMD_ARM_SETTLE){
      return ArmSubsystem.get().atSetpoint();
    }
    return true;
  }
  */
  // motor does more than a revolution for 42
    if(encoderoutput1 >= 15 && encoderoutput2 >= 15 && encoderoutput3 >= 15 && encoderoutput4 >= 15) {
      System.out.println("Stopped");
      return true;


    }
    else{
    System.out.println("isFinished()");
    return false;
}
}
}