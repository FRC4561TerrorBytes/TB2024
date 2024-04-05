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
    leds = new AddressableLED(4);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
    loadingNotifier =
      new Notifier(
        () -> {
          synchronized (this) {
            breath(Color.kWhite, Color.kBlack, System.currentTimeMillis() / 1000.0);
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
    double txAngle = Math.abs(tx.getDouble(57));

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

    // Select LED mode
    solid(Color.kBlack); // Default off
    if (estopped) {
      solid(Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        solid(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime), Color.kGreen);
       } else if (prideLeds) {
        // Pride stripes
        stripes(
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
        wave(allianceColor, secondaryDisabledColor, waveAllianceCycleLength, waveAllianceDuration);
      }
    }
      if (DriverStation.isAutonomous()) {
        wave(Color.kDarkGreen, Color.kHotPink, waveSlowCycleLength, waveSlowDuration);
        if (autoFinished) {
          double fullTime = (double) length / waveFastCycleLength * waveFastDuration;
          solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
        }
      } else { // Enabled
        if (intaking) {
          strobe(Color.kDodgerBlue, strobeSlowDuration);
        } else if (autoShootCommand) {
          solid(1.0 - (txAngle / length), Color.kPurple);
        } else if (autoShoot && (id == 7.0 || id == 4.0)) {
          rainbow(rainbowCycleLength, rainbowDuration);
        } else if (autoShoot && id == 0.0) {
          strobe(Color.kRed, strobeSlowDuration);
        }else if (noteInIndexer) {
          solid(Color.kDarkOrange);
        } else if (noteInIntake) {
          solid(Color.kLawnGreen);
        }
      }

    leds.setData(buffer);
  }

  private void solid(Color color) {
    if (color != null) {
      for (int i = 0; i < length; i++) {
        buffer.setLED(i, color);
      }
    }
  }

  private void solid(double percent, Color color) {
    for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
      buffer.setLED(i, color);
    }
  }

  private void strobe(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(c1On ? c1 : c2);
  }

  private void strobe(Color color, double duration) {
    strobe(color, Color.kBlack, duration);
  }

  private void breath(Color c1, Color c2) {
    breath(c1, c2, Timer.getFPGATimestamp());
  }

  private void breath(Color c1, Color c2, double timestamp) {
    double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }

  private void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < length; i++) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < length; i++) {
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

  private void stripes(List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = 0; i < length; i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }
}
