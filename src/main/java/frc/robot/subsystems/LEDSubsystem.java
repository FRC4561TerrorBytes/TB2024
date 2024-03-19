package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    
    public LEDSubsystem() {
        m_led = new AddressableLED(9);
        m_ledBuffer = new AddressableLEDBuffer(50);

        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++)
        {
            m_ledBuffer.setRGB(i,r,g,b);
        }
        m_led.setData(m_ledBuffer);
    }
}