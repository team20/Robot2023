// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Status;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArduinoSubsystem extends SubsystemBase {
  private I2C i2c = new I2C(Port.kMXP, 0x18);
  /** Creates a new ArduinoSubsystem. */

  private byte[] m_statusCode = new byte[1];

  private enum StatusCode{ 
    DEFAULT_OR_TEAMCOLOR_OR_ALLIANCECOLOR((byte)9),
    ORANGE_THEATER_LIGHTS((byte)10),
    BLUE_THEATER_LIGHTS((byte)11),   
    RED_THEATER_LIGHTS((byte)12),
    MOVING_GREEN_AND_BLUE_GRADIENT((byte)13),
    GREEN_THEATER_LIGHTS((byte)14),
    MOVING_RED_AND_GREEN_GRADIENT((byte)15),
    BLUE_BACK_AND_FORTH_TIMER((byte)16), // TODO: timer currently has a bug (look in arduino IDE)
    GREEN_BACK_AND_FORTH_TIMER((byte)17),
    PURPLE_BLINKING((byte)18),
    YELLOW_BLINKING((byte)19);
    
    public byte code;
    private StatusCode(byte c){
      code = c; 
    }
  }
  public ArduinoSubsystem() {
    setCode(StatusCode.DEFAULT_OR_TEAMCOLOR_OR_ALLIANCECOLOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    i2c.writeBulk(m_statusCode);
  }
  public void setCode(StatusCode code){
    m_statusCode[0] = code.code;
  }
}
