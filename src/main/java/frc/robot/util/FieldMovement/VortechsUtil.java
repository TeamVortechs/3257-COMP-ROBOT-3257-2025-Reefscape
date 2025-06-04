package frc.robot.util.FieldMovement;

public class VortechsUtil {
  public static double clamp(double num, double clampVal) {
    return clamp(num, -clampVal, clampVal);
  }

  public static double clamp(double num, double lowerClamp, double higherClamp) {
    return Math.max(lowerClamp, Math.min(num, higherClamp));
  }
}
