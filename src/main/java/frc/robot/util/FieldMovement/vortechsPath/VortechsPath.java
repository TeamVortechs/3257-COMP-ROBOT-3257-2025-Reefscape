package frc.robot.util.FieldMovement.vortechsPath;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

//This is how we store the end position of the robot and how we add the extra functionality with commands.
public abstract class VortechsPath {

    //the target pose that is actually being pathfinded to
    private int selectedTargetPose = 0;

    protected Supplier<Pose2d> robotPoseSupplier;

    public VortechsPath(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }
        
    //gets the endpoint of the path. The robot will automatically pathfinding to this 
    public abstract Pose2d[] getTargetPoses();

    //sets the target pose that is actualy being pathfinded to. This is handled the in the pathfinder manager.
    public void setSelectedTargetPose(int targetPose) {
        this.selectedTargetPose = targetPose;
    }

    //extra functionality 
    public abstract void tick(Pose2d robotPose);

    //each different group of paths has a different pipeline. For example a coral scorign path would have a different pipeline then an algae shooting path. The robot goes to the closest path of a specific pipeline
    public abstract int getPipeline();

    //the commands that will be run while the robot pathfinds. This is usually how it does extra util
    public abstract Command getParallelCommandGroup();

    //helper method that gets the distance from the target pose
    public double getDistance() {
        return robotPoseSupplier.get().getTranslation().getDistance(getTargetPoses()[selectedTargetPose].getTranslation());
    }
}