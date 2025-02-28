// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

public class KDoubleBool {
  private boolean defaultValue;
  private String name;

  public KDoubleBool(String name, boolean defaultValue) {
    this.defaultValue = defaultValue;
    this.name = name;

    Preferences.initBoolean(name, defaultValue);
  }

  public boolean getValue() {
    return Preferences.getBoolean(name, defaultValue);
  }

  public void setValue(boolean value) {
    Preferences.setBoolean(name, value);
  }
}
