// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Topic;

import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOLimelightSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  private final NetworkTableInstance inst;
  private final NetworkTable LLTable;
  private final IntegerPublisher tvPub;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOLimelightSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;
        inst = NetworkTableInstance.getDefault();
        LLTable =  inst.getTable(name);

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var cameraProp = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProp);

    // Create simulated camera properties. These can be set to mimic your actual camera.
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(60);
    cameraProp.setAvgLatencyMs(10);
    cameraProp.setLatencyStdDevMs(3);

    // Add the simulated camera to view the targets on this simulated field.
    visionSim.addCamera(cameraSim, robotToCamera);

    // add simulated limelight variables
    IntegerTopic tv = LLTable.getIntegerTopic("tv");
    tvPub = tv.publish();
    tvPub.setDefault(0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
    // tvPub.set();
  }
}
