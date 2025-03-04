// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

public class KDoublePref {
  private double defaultValue;
  private String name;

  public KDoublePref(String name, double defaultValue) {
    this.defaultValue = defaultValue;
    this.name = name;

    Preferences.initDouble(name, defaultValue);
  }

  public double getValue() {
    return Preferences.getDouble(name, defaultValue);
  }

  public void setValue(double value) {
    Preferences.setDouble(name, value);
  }
}
