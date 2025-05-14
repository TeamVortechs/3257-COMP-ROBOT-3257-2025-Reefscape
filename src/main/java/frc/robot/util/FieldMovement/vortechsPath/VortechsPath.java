package frc.robot.util.FieldMovement.vortechsPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

//This is how we store the end position of the robot and how we add the extra functionality with commands.
public interface VortechsPath {
    //gets the endpoint of the path. The robot will automatically pathfinding to this 
    Pose2d getTargetPose();

    //extra functionality 
    void tick(Pose2d robotPose);

    //each different group of paths has a different pipeline. For example a coral scorign path would have a different pipeline then an algae shooting path. The robot goes to the closest path of a specific pipeline
    int getPipeline();

    //the commands that will be run while the robot pathfinds. This is usually how it does extra util
    Command getParallelCommandGroup();
}