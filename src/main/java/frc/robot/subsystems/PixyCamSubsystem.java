package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArduinoConstants;

public class PixyCamSubsystem extends SubsystemBase {
    private static PixyCamSubsystem s_subsystem;
    private I2C i2c = new I2C(Port.kMXP, ArduinoConstants.kPixyCamAddress);
    private byte[] m_buffer = new byte[7];

    /** Creates a new PixyCamSubsystem. */
    public PixyCamSubsystem() {
        // Singleton
        if (s_subsystem != null) {
            try {
                throw new Exception("PixyCam subsystem already initialized!");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        s_subsystem = this;
    }

    public static PixyCamSubsystem get() {
        return s_subsystem;
    }

    int m_x() {
        return m_buffer[0] & 0xFF << 8 | m_buffer[1] & 0xFF;
    }

    int m_y(){
        return m_buffer[2];
    }

    int m_width(){
        return m_buffer[3] & 0xFF | m_buffer[4] & 0xFF;
    }
    
    int m_height(){
        return m_buffer[5];
    }
    
    int m_signature(){
        return m_buffer[6];
    }



    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        i2c.read(ArduinoConstants.kPixyCamAddress, 7, m_buffer);
        SmartDashboard.putNumber("PixyCam X", m_x());
        SmartDashboard.putNumber("PixyCam Y", m_y());
        SmartDashboard.putNumber("PixyCam Width", m_width());
        SmartDashboard.putNumber("PixyCam Height", m_height());
        SmartDashboard.putNumber("PixyCam Signature", m_signature());

    }

    // public void setCode(StatusCode code) {
    // m_statusCode[0] = code.code;
    // }
}
