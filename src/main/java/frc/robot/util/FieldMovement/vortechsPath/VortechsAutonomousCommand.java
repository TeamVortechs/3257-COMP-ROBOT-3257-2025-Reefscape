// package frc.robot.util.FieldMovement.vortechsPath;

// import java.util.function.Supplier;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;

// //This is how we store the end position of the robot and how we add the extra functionality with
// commands.
// public abstract class VortechsAutonomousCommand {

//     //the target pose that is actually being pathfinded to
//     private int selectedTargetPose = 0;

//     protected Supplier<Pose2d> robotPoseSupplier;

//     public VortechsAutonomousCommand(Supplier<Pose2d> robotPoseSupplier) {
//         this.robotPoseSupplier = robotPoseSupplier;
//     }

//     //extra functionality in case it is needed
//     public abstract void tick(Pose2d robotPose);

//     //each different group of paths has a different pipeline. For example a coral scorign path
// would have a different pipeline then an algae shooting path. The robot goes to the closest path
// of a specific pipeline
//     public abstract int getPipeline();

//     //the commands that will be run while the robot pathfinds. This is usually how it does extra
// util
//     public abstract Command getParallelCommandGroup();

//     //wether or not the path should be flipped
//     public abstract boolean isFlipped();

//     //logic for the target pose selection and selected target pose
//     //gets the endpoint of the path. The robot pathfinder will automatically pathfind to the
// closest one of these paths
//     public abstract Pose2d[] getTargetPoses();

//     //gets the value of the path. Lower value paths
//     public abstract double getValue();

//     //sets the target pose that is actualy being pathfinded to. This is handled the in the
// pathfinder manager.
//     public void setSelectedTargetPose(int targetPose) {
//         this.selectedTargetPose = targetPose;
//     }

//     //helper methods
//     //gets the distance from the pose to the closest target pose
//     public double getDistance(Pose2d pose) {
//         double shortestDistance = Double.MAX_VALUE;

//         for(int i = 0; i < getTargetPoses().length; i++) {
//             double distance =
// getTargetPoses()[i].getTranslation().getDistance(pose.getTranslation());

//             shortestDistance = Math.min(shortestDistance, distance);
//         }

//         return shortestDistance;
//     }

//     //helper method that gets the distance from the target pose
//     public double getDistance() {
//         return
// robotPoseSupplier.get().getTranslation().getDistance(getTargetPoses()[selectedTargetPose].getTranslation());
//     }
// }
