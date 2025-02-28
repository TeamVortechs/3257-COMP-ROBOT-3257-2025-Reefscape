package frc.robot;

import frc.robot.util.KDoublePref;

// these preferences are able to be changed at runtime. Most likely this stuff will just be used for
// faster PID loop tuning
public class KDoublePreferences {
  public static class PDrivetrain {
    public static final KDoublePref kPTranslation = new KDoublePref("trans proportional", 0.45);
    public static final KDoublePref kITranslation = new KDoublePref("trans integral", 0.6);
    public static final KDoublePref kDTranslation = new KDoublePref("trans derivative", 0);

    public static final KDoublePref kPRotation =
        new KDoublePref(
            "rotation proportional",
            0.06); // .02 is our autoturntofacetarget value; change this to like 0.10 in the case
    // that we do not get a kD value locked in
    public static final KDoublePref kIRotation =
        new KDoublePref(
            "integral rotation",
            0.000625); // .04 is our autoturntofacetarget value; change this to 0 in the case that
    // we do not get a kD value locked in
    public static final KDoublePref kDRotation =
        new KDoublePref(
            "derivative rotation",
            0.0); // 0.015 // this isn't done. this value must be tweaked more; if this is not
    // locked in, change to 0
  }

  public static class PElevator {
    public static KDoublePref kG =
        new KDoublePref("elevator kG", 0.2); // Add kG V output to overcome gravity
    public static KDoublePref kS =
        new KDoublePref("elevator kS", 0.25); // Add kS V output to overcome static friction
    public static KDoublePref kV =
        new KDoublePref("elevator kV", 0.12); // A velocity target of 1 rps results in kV V output
    public static KDoublePref kA =
        new KDoublePref("elevator kA", 0.01); // An acceleration of 1 rps/s requires kA V output
    public static KDoublePref kP =
        new KDoublePref(
            "elevator kP", 7.5); // A position error of 1 rotation results in kP V output
    public static KDoublePref kI =
        new KDoublePref("elevator kI", 0); // no output for error over time
    public static KDoublePref kD =
        new KDoublePref("elevator kD", 0.01); // A velocity error of 1 rps results in 0.1 V output

    public static KDoublePref speedLimit =
        new KDoublePref("elevator velocity limit", 15); // unit is rotation per second
    public static KDoublePref accelerationLimit =
        new KDoublePref("elevator accel limit", 13); // unit is rotation/second^2
    public static KDoublePref jerkLimit =
        new KDoublePref("elevator jerk limit", 30); // unit is rotation/second^3
    public static KDoublePref tolerance = new KDoublePref("Elevator Target Tolerance", 0.1);

    public static KDoublePref manualSpeedLimit =
        new KDoublePref("elevator MANUAL speed limit", 0.3); // range of 0 to 1
  }

  public static class PWrist {
    public static KDoublePref kS =
        new KDoublePref("wrist kS", 0.25); // Add kS V output to overcome static friction
    public static KDoublePref kV =
        new KDoublePref("wrist kV", 0.12); // A velocity target of 1 rps results in kV V output
    public static KDoublePref kA =
        new KDoublePref("wrist kA", 0.01); // An acceleration of 1 rps/s requires kA V output
    public static KDoublePref kP =
        new KDoublePref("wrist kP", 4.8); // A position error of 1 rotation results in kP V output
    public static KDoublePref kI = new KDoublePref("wrist kI", 0); // no output for error over time
    public static KDoublePref kD =
        new KDoublePref("wrist kD", 0.1); // A velocity error of 1 rps results in 0.1 V output

    public static KDoublePref speedLimit = new KDoublePref("wrist velocity limit", 3);
    public static KDoublePref accelerationLimit = new KDoublePref("wrist accel limit", 2.5);
    public static KDoublePref jerkLimit = new KDoublePref("wrist jerk limit", 10);
    public static KDoublePref tolerance = new KDoublePref("Wrist Target Tolerance", 0.1);

    public static KDoublePref manualSpeedLimit = new KDoublePref("wrist MANUAL speed limit", 0.3);
  }

  public static class PCamera {
    public static KDoublePref xOffset = new KDoublePref("Camera X offset", 0.2286);
    public static KDoublePref yOffset = new KDoublePref("Camera Y offset", -0.2002028);
    public static KDoublePref zOffset = new KDoublePref("Camera Z offset", 0.739902);
    public static KDoublePref rotationRoll = new KDoublePref("Camera rotation roll offset", 0);
    public static KDoublePref rotationPitch =
        new KDoublePref("Camera rotation pitch offset", 0.174533);
    public static KDoublePref rotationYaw = new KDoublePref("Camera rotation yaw offset", 0.349066);
  }
}
