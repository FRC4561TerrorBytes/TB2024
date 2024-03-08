package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A singleton instance of this class holds the current robot state with regard
 * to game objectives such as the current game piece mode.
 */
public class GameMode extends SubsystemBase{

  /** The singleton instance. */
  private static final GameMode INSTANCE = new GameMode();

  /** Mode enum */
  public enum Mode {
    SPEAKER,
    AMP,
    TRAP,
    INTAKING,
    IDLE;
  }

  /** Default mode is speaker */
  private Mode currentMode = Mode.SPEAKER;

  /** private for singleton instance */
  private GameMode() {}

  /** 
   * Obtain single instance
   * 
   * @return the single {@link GameMode} instance
   */
  public static GameMode getInstance() {
    return INSTANCE;
  }

  /**
   * @return The current {@link Mode} we are in
   */
  public Mode getCurrentMode() {
    return currentMode;
  }

  /**
   * Changes the current mode
   * 
   * @param mode The current mode 
   */
  public void setCurrentMode(final Mode mode) {
    currentMode = mode;
  }

  public void periodic() {
    Logger.recordOutput("GameMode/Mode", currentMode);
  }
}