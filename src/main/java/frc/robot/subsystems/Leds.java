// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {

  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  //Robot States
  public int loopCycleCount = 0;
  public boolean intaking = false;
  public boolean noteInIntake = false;
  public boolean noteInIndexer = false;
  public boolean autoFinished = false;
  public boolean autoShoot = false;
  public boolean autoDrive = false;
  public double autoFinishedTime = 0.0;
  public boolean autoShootCommand = false;
  public boolean autoNoteAlign = false;
  public double autoShootStartAngle = 0.0;
  public boolean canDisconnect = false;
  public boolean firmwareAlert = false;
  public boolean currentAlert = false;
  public boolean staticOn = false;

  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kBlack;
  private Color secondaryDisabledColor = Color.kDeepPink;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  //Led IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  private final Notifier loadingNotifier;

  //Constants
  private static final boolean prideLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final int length = 57;
  private static final int staticLength = 7;
  private static final double strobeFastDuration = 0.1;
  private static final double strobeSlowDuration = 0.4;
  private static final double breathDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveSlowCycleLength = 25.0;
  private static final double waveSlowDuration = 1.0;
  private static final double waveAllianceCycleLength = 15.0;
  private static final double waveAllianceDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal


  public Leds() {
    leds = new AddressableLED(9);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();

    loadingNotifier =
      new Notifier(
        () -> {
          synchronized (this) {
            breath(Section.FULL, Color.kWhite, Color.kBlack, System.currentTimeMillis() / 1000.0);
            leds.setData(buffer);
          }
        });
    loadingNotifier.startPeriodic(0.02);
  }

  public synchronized void periodic() {
    Logger.recordOutput("LEDS/Auto Shoot", autoShoot);

    NetworkTable chair = NetworkTableInstance.getDefault().getTable("limelight-vanap");
    NetworkTableEntry tx = chair.getEntry("tx");
    NetworkTableEntry tid = chair.getEntry("tid");
    double id = tid.getDouble(0.0);
    double txAngle = Math.abs(tx.getDouble(57)) - 2;

    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor = 
        alliance
          .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
          .orElse(Color.kBlack);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : Color.kDeepPink;
    }

    // Update auto state
    if (DriverStation.isDisabled()) {
      autoFinished = false;
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    if (canDisconnect || firmwareAlert || currentAlert) {
      staticOn = true;
    } else {
      staticOn = false;
    }

    // Select LED mode
    solid(Section.FULL, Color.kBlack); // Default off
    if (estopped) {
      solid(Section.FULL, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        solid(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime), Color.kGreen);
      } else if (staticOn) {
        wave(Section.NONSTATIC, allianceColor, secondaryDisabledColor, waveAllianceCycleLength, waveAllianceDuration);
        if (canDisconnect) {
          strobe(Section.STATIC, Color.kDarkRed, strobeSlowDuration);
        } else if (firmwareAlert) {
          strobe(Section.STATIC, Color.kBlue, strobeSlowDuration);
        } else if (currentAlert) {
          strobe(Section.STATIC, Color.kLightYellow, strobeSlowDuration);
        }
      } else if (prideLeds) {
        // Pride stripes
        stripes(
          Section.FULL,
            List.of(
                Color.kBlack,
                Color.kRed,
                Color.kOrangeRed,
                Color.kYellow,
                Color.kGreen,
                Color.kBlue,
                Color.kPurple,
                Color.kBlack,
                new Color(0.15, 0.3, 1.0),
                Color.kDeepPink,
                Color.kWhite,
                Color.kDeepPink,
                new Color(0.15, 0.3, 1.0)),
            50,
            5.0);
        buffer.setLED(staticLength, allianceColor);
      } else {
        // Default pattern
        wave(Section.FULL, allianceColor, secondaryDisabledColor, waveAllianceCycleLength, waveAllianceDuration);
      }
    } else if (DriverStation.isAutonomous()) {
        wave(Section.FULL, Color.kDarkGreen, Color.kHotPink, waveSlowCycleLength, waveSlowDuration);
        if (autoFinished) {
          double fullTime = (double) length / waveFastCycleLength * waveFastDuration;
          solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
        }
      } else { // Enabled
        if (autoNoteAlign) {
          strobe(Section.FULL, Color.kDarkOrange, strobeSlowDuration);
        } else if (intaking) {
          strobe(Section.FULL, Color.kDodgerBlue, strobeSlowDuration);
        } else if (autoShootCommand) {
          solid(1.0 - (txAngle / autoShootStartAngle), Color.kPurple);
        } else if (autoShoot && (id == 7.0 || id == 4.0)) {
          rainbow(Section.FULL, rainbowCycleLength, rainbowDuration);
        } else if (autoShoot && (id != 7.0 || id != 4.0)) {
          strobe(Section.FULL, Color.kRed, strobeSlowDuration);
        } else if (noteInIndexer) {
          solid(Section.FULL, Color.kOrangeRed);
        } else if (noteInIntake) {
          solid(Section.FULL, Color.kLawnGreen);
        }
      }

    leds.setData(buffer);
  }

  private void solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void solid(double percent, Color color) {
    for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
      buffer.setLED(i, color);
    }
  }

  private void strobe(Section section, Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, c1On ? c1 : c2);
  }

  private void strobe(Section section, Color color, double duration) {
    strobe(section, color, Color.kBlack, duration);
  }

  private void breath(Section section, Color c1, Color c2) {
    breath(section, c1, c2, Timer.getFPGATimestamp());
  }

  private void breath(Section section, Color c1, Color c2, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(Section section, List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = section.start(); i < section.end(); i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  public void clearAlerts() {
    currentAlert = false;
    firmwareAlert = false;
    canDisconnect = false;
  }

  public static enum Section {
    STATIC,
    NONSTATIC,
    FULL;

    private int start() {
      switch (this) {
        case STATIC:
          return length - staticLength; 
        case NONSTATIC:
          return 0;
        case FULL:
          return 0;   
        default:
          return 0;
      }
    }

    private int end() {
      switch (this) {
        case STATIC:
          return length;
        case NONSTATIC:
          return length - staticLength;
        case FULL:
          return length;      
        default:
          return length;
      }
    }

  }

}
