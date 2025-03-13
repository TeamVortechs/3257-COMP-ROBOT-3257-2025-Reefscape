package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class IntakeHomingVortechs {
    private Pose2d gamePieceLoc;
    private Drive drive;

    private final double kp = 1;

    public IntakeHomingVortechs(Drive drive) {
        this.drive = drive;
        gamePieceLoc = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }

    public void setGamepieceLocation(Pose2d gamePieceLoc) {
        this.gamePieceLoc = gamePieceLoc;
    }

    public ChassisSpeeds getNoteOffsetVector() {
        double distance;

        double xoffset;

        return 
    }
}
