package frc.robot.util.FieldMovement;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.function.Supplier;

public class ClosestPoseSupplierVortechs {
  private List<Pose2d>[] targetPoses;
  private Supplier<Pose2d> drivetrainPoseSupplier;
  private int pipeline = 0;

  public ClosestPoseSupplierVortechs(
      Supplier<Pose2d> drivetrainPoseSupplier, List<Pose2d>... targetPoses) {
    this.drivetrainPoseSupplier = drivetrainPoseSupplier;
    this.targetPoses = targetPoses;
  }

  public void setPipeline(int index) {
    this.pipeline = index;
  }

  public int getPipeline() {
    return pipeline;
  }

  public Pose2d getClosestPose() {
    double closestDistance = Double.MAX_VALUE;
    Pose2d closestPose = new Pose2d();

    Pose2d curPose = drivetrainPoseSupplier.get();

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      curPose = FlippingUtil.flipFieldPose(curPose);
    }

    for (Pose2d targetPose : targetPoses[pipeline]) {
      double testDist = curPose.getTranslation().getDistance(targetPose.getTranslation());

      if (testDist < closestDistance) {
        closestDistance = testDist;
        closestPose = targetPose;
      }
    }

    return closestPose;
  }
}
