// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SelfCheck;

import edu.wpi.first.wpilibj.Timer;
import java.util.Objects;

/** Add your docs here. */
public class SubSystemFaults {
    public String description;
    public double timestamp;
    public boolean isWarning;
    public boolean sticky;

    public SubSystemFaults(String description, boolean isWarning) {
        // default sticky to true
        this(description, isWarning, true);
      }
    
      public SubSystemFaults(String description) {
        this(description, false);
      }
    
      public SubSystemFaults(String description, boolean isWarning, boolean sticky) {
        this.description = description;
        this.timestamp = Timer.getFPGATimestamp();
        this.isWarning = isWarning;
        this.sticky = sticky;
      }
    

      @Override
  public boolean equals(Object comparison) {
    if (this == comparison) {
      return true;
    }

    if (comparison == null) {
      return false;
    }

    if (getClass() != comparison.getClass()) {
      return false;
    }

    SubSystemFaults otherSubsystemFault = (SubSystemFaults) comparison;

    return description.equals(otherSubsystemFault.description)
        && isWarning == otherSubsystemFault.isWarning;
  }

  @Override
  public int hashCode() {
    return Objects.hash(description, timestamp, isWarning, sticky);
  }
}
