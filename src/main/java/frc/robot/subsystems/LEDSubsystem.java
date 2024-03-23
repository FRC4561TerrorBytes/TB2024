package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.rgbValues;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    
    public LEDSubsystem() {
        m_led = new AddressableLED(4);
        m_ledBuffer = new AddressableLEDBuffer(57);

        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();

        setColor(rgbValues.GREEN);
    }   

    public Command IntakingLedCycle = new CycleColor(rgbValues.FUNNY_COLOR, 999.0)
      .andThen(new WaitCommand(0.25))
      .andThen(new ScheduleCommand(new InstantCommand(() -> setColor(rgbValues.FUNNY_COLOR))));

    public Command ShootingLedCycle = new CycleColor(rgbValues.PURPLE_SHOOT, 10.0)
      .andThen(new WaitCommand(0.25))
      .andThen(new ScheduleCommand(new InstantCommand(() -> setColor(rgbValues.PURPLE_SHOOT))));

    public void setColor(rgbValues rgb) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++)
        {
            m_ledBuffer.setRGB(i, rgb.r, rgb.g, rgb.b);
        }
        m_led.setData(m_ledBuffer);
    }

    public boolean flashColor(rgbValues rgb, int count) {
      for (int i = 0; i < count; i++) {
          setColor(rgb);
          for (int x = 0; x < 24; x++) {}
          setColor(rgbValues.BLANK);
          for (int y = 0; y < 24; y++) {}
      }
      return true;
    }

    public boolean flashColorThenSolid(rgbValues rgb, int count) {
      if (flashColor(rgb, count)) {
        setColor(rgb);
      }
      return true;
    }

    /**
   * A nested command class used to cycle the game piece lights.
   */
  public class CycleColor extends Command {
    private final int m_r;
    private final int m_g;
    private final int m_b;
    private final Timer m_timer = new Timer();
    private int m_lightsOn;
    private double timeout;

    /** Creates a new LEDCycleFront. */
    public CycleColor(rgbValues rgb, double  timeOut) {
      m_r = rgb.r;
      m_g = rgb.g;
      m_b = rgb.b;
      this.timeout = timeOut;
      
      addRequirements(LEDSubsystem.this);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_timer.reset();
      m_timer.start();
      m_lightsOn = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        if (i <= m_lightsOn) {
          m_ledBuffer.setRGB(i, m_r, m_g, m_b);
        } else {
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
      }
      m_led.setData(m_ledBuffer);

      m_lightsOn++;
      if (m_lightsOn >= m_ledBuffer.getLength()) {
        m_lightsOn = 0;
      }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(timeout);
    }
  }
}