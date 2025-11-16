package frc.robot.subsystems.vision.Detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.wrist.WristIOInputsAutoLogged;

import org.littletonrobotics.junction.AutoLog;

public interface DetectionIO {
  @AutoLog
  public static class DetectionIOInputs {
    Pose2d closestDetectionPose = null;
    boolean isDetected = false;
    int numberOfObjects = 0;
    double objectDistance = 0;
  }

  public default void updateInputs(DetectionIOInputsAutoLogged inputs) {}

  public default void update() {}
  ;

  public default Pose2d getObjectPosition() {
    return new Pose2d(new Translation2d(), new Rotation2d());
  }

  public default boolean isDetected() {
    return false;
  }

  public default void setPipeline(int pipeline) {}
  ;
}
