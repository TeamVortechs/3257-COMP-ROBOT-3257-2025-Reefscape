package frc.robot.commandautos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.autoCommands.DriveCommands;
import frc.robot.commands.wrist.SetWristRollerSpeedCommand;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class midStartOneAlgae extends SequentialCommandGroup {

  public midStartOneAlgae(Wrist wrist, Elevator elevator, Drive drive, Alliance allianceColor) {
    addRequirements(wrist, elevator, drive);

    final boolean isBlue = allianceColor == Alliance.Blue ? true : false;
    final double driveVelocity = 0.4;

    final double aimAngle =
        isBlue
            ? Math.PI / 4
            : Math.PI
                + Math.PI / 4; // have to flip the angle
    if (isBlue) {
      drive.setPose(new Pose2d(7.2, 4.000, new Rotation2d(Math.PI)));
    } else {
      drive.setPose(new Pose2d(10.35, 4.000, new Rotation2d()));
    }

    addCommands(

        // set elevator and arm position for LOW ALGAE
        // ScoringCommands.prepForScoring(1, wrist, elevator),
        // slowly drive forwards
        // future: put canrange facing front of robot to detect range from reef; will enable going
        // fast and then using PID for distance
        new PrintCommand("I am beginning the autonomous."),
        DriveCommands.joystickDrive(drive, () -> -driveVelocity, () -> 0, () -> 0)
            .withTimeout(1), // find this out with testing
        // before getting to the reef, enable intaking
        new PrintCommand("Setting wrist roller speed to intake."),
        // new ScheduleCommand(
        //     new SetWristRollerSpeedCommand(wrist, 0.6)
        //         .withTimeout(1)), // keep the command rolling while the timeout runs
        // have arm drive into the algae, then stop intake after 1 second (assume successful)
        // future: put canrange in the arm to detect success
        DriveCommands.joystickDrive(drive, () -> (-driveVelocity * 0.8), () -> 0, () -> 0)
            .withTimeout(0.6),
        // drive directly backwards until back of bumpers touches the starting line
        DriveCommands.joystickDrive(drive, () -> (driveVelocity), () -> 0, () -> 0)
            .withTimeout(1.3),
        // turn about 135 degrees
        DriveCommands.joystickDriveAtAngle(drive, () -> 0, () -> 0, () -> new Rotation2d(aimAngle))
            .withTimeout(1),
        // raise arm up and extend elevator up to BARGE LEVEL
        new PrintCommand("Setting arm to scoring."),
        // ScoringCommands.prepForScoring(3, wrist, elevator).andThen(new WaitUntilCommand(() ->
        // elevator.isOnTarget())),
        // fully raise arm to scoring position
        new PrintCommand("Fully raised arm."),
        // new SetWristTargetAngleCommand(wrist, () -> 0).andThen(new WaitUntilCommand(() -> wrist.isOnTarget())),
        // eject algae
        new PrintCommand("Ejected algae."),
        // new SetWristRollerSpeedCommand(wrist, 1).withTimeout(1),
        // bring elevator back down
        new PrintCommand("Bringing elevator back down.")
        // ScoringCommands.prepForScoring(4, wrist, elevator)
        );
  }
}
