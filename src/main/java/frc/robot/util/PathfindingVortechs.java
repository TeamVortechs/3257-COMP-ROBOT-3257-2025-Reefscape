package frc.robot.util;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.FieldMovement.LocalADStarAK;

public class PathfindingVortechs {

/*
new algorithm:
copy matt's comment
 */

    private LocalADStarAK pathfinder;
    private PathConstraints pathConstraints;

    private Pose2d targetPose;
    private PathPlannerPath pathInEffect;

    private Supplier<Pose2d> robotPose;
    private Supplier<ChassisSpeeds> speeds;
    private RobotConfig robotConfig;

    public PathfindingVortechs(LocalADStarAK pathfinder, PathConstraints pathConstraints, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> speeds, RobotConfig robotConfig) {
        this.pathfinder = pathfinder;
        this.pathConstraints = pathConstraints;

        this.robotPose = robotPose;
        this.speeds = speeds;
        this.robotConfig = robotConfig;

        
    }

    public void testLog() {
         
        }

    //converts the given controller values into ones that serve the pathfinder. This is the primary way that pathfinder will update the robot position
    public void getIdealVector(Vector vector) {
        PathPlannerTrajectory pathPlannerTrajectory = pathInEffect.generateTrajectory(speeds.get(), robotPose.get().getRotation(), robotConfig);
        AutoBuilder.followPath(pathInEffect);
    }

    //sets the target pose of the path
    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        pathfinder.setGoalPosition(targetPose.getTranslation());

        //updates the values so all the needed systems use the given constraints
        updateValues();
    }

    //sets the constraints of the path in action
    public void setPathConstraints(PathConstraints pathConstraints) {
        this.pathConstraints = pathConstraints;
        
        //updates the values so all the needed systems use the given constraints
        updateValues();
    }

    private void updateValues() {

        if(pathfinder.isNewPathAvailable()) {
            pathInEffect = pathfinder.getCurrentPath(pathConstraints, new GoalEndState(0,  targetPose.getRotation()));
        }
    }

    public class Vector {
        private double x;
        private double y;

        public Vector(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }
    }
}
