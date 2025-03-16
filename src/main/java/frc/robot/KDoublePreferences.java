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
    public static double kG = 0; // Add kG V output to overcome gravity
    public static double kS = 0; // Add kS V output to overcome static friction
    public static double kV = 0; // A velocity target of 1 rps results in kV V output
    public static double kA = 0; // An acceleration of 1 rps/s requires kA V output
    public static double kP = 0; // A position error of 1 rotation results in kP V output
    public static double kI = 0; // no output for error over time
    public static double kD = 0; // A velocity error of 1 rps results in 0.1 V output

    public static double speedLimit = 0;
    // unit is rotation per second
    public static double accelerationLimit = 0;
    public static double jerkLimit = 0;
    public static double tolerance = 0;
    public static double manualSpeedLimit = 0; // range of 0 to 1
  }

  public static class PWrist {
    public static KDoublePref kS =
        new KDoublePref("wrist kS", 0.25); // Add kS V output to overcome static friction
    public static KDoublePref kV =
        new KDoublePref("wrist kV", 0.12); // A velocity target of 1 rps results in kV V output
    public static KDoublePref kA =
        new KDoublePref("wrist kA", 0.01); // An acceleration of 1 rps/s requires kA V output
    public static KDoublePref kP =
        new KDoublePref("wrist kP", 7.5); // A position error of 1 rotation results in kP V output
    public static KDoublePref kI = new KDoublePref("wrist kI", 0); // no output for error over time
    public static KDoublePref kD =
        new KDoublePref("wrist kD", 0.1); // A velocity error of 1 rps results in 0.1 V output
    public static KDoublePref kG =
        new KDoublePref("wrist kG", 0.2); // free kP for gravity but like, arm-y

    public static KDoublePref speedLimit = new KDoublePref("wrist velocity limit", 3);
    public static KDoublePref accelerationLimit = new KDoublePref("wrist accel limit", 2.5);
    public static KDoublePref jerkLimit = new KDoublePref("wrist jerk limit", 10);
    public static KDoublePref tolerance = new KDoublePref("Wrist Target Tolerance", 0.1);

    public static KDoublePref manualSpeedLimit = new KDoublePref("wrist MANUAL speed limit", 0.3);
  }
}
