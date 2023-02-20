// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArduinoConstants;

public class ArduinoSubsystem extends SubsystemBase {
  private I2C i2c = new I2C(Port.kMXP, 0x18);
  /** Creates a new ArduinoSubsystem. */

  private byte[] m_statusCode = new byte[1];

  public enum StatusCode {
    RESET((byte) 8),
    BLINKING_YELLOW((byte) 9),
    BLINKING_PURPLE((byte) 10),
    MOVING_GREEN_AND_RED_GRADIENT((byte) 11),
    MOVING_GREEN_AND_BLUE_GRADIENT((byte) 12),
    DEFAULT_OR_TEAMCOLOR_OR_ALLIANCECOLOR((byte) 20);

    public byte code;

    private StatusCode(byte c) {
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

  public void setCode(StatusCode code) {
    m_statusCode[0] = code.code;
  }
}
