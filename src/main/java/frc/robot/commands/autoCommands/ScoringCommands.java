package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.elevator.SetElevatorPresetCommand;
import frc.robot.commands.wrist.SetWristRollerSpeedCommand;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class ScoringCommands {

  public static Command scoreAuto(int level, Wrist wrist, Elevator elevator) {
    return new InstantCommand(() -> wrist.setRollerSpeed(0.2), wrist)
        .andThen(
            new SetElevatorPresetCommand(
                    elevator,
                    wrist,
                    Constants.Elevator.MAX_HEIGHT - 0.1) // just a bit less than max
                .alongWith(
                    new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.WRIST_STAGE_4_ANGLE)))
        .andThen(new SetWristRollerSpeedCommand(wrist, 1));
  }

  public static Command prepForScoring(int level, Wrist wrist, Elevator elevator) {
    switch (level) { // bit of a misnomer here

      case 1: // low reef algae
        return new InstantCommand(() -> wrist.setRollerSpeed(0.2), wrist)
          .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.WRIST_STAGE_2_ANGLE))
          .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
          .andThen(new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.STAGE_2_LEVEL));

      case 2: // high reef algae
        return new InstantCommand(() -> wrist.setRollerSpeed(0.2), wrist)
          .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.WRIST_STAGE_2_ANGLE))
          .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
          .andThen(new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.STAGE_3_LEVEL));

      case 3: // scoring position
        return new InstantCommand(() -> wrist.setRollerSpeed(0.2), wrist)
          .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.WRIST_STAGE_4_ANGLE))
          .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
          .andThen(new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.BARGE_LEVEL));

      case 4: // minimum height
        return new InstantCommand(() -> wrist.setRollerSpeed(0.2), wrist)
          .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.WRIST_STAGE_4_ANGLE))
          .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
          .andThen(new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.MIN_HEIGHT));
      
      case 5: // processor position
        return new InstantCommand(() -> wrist.setRollerSpeed(0.2), wrist)
          .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.WRIST_PROCESSOR_ANGLE))
          .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
          .andThen(new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.MIN_HEIGHT));

      default: // oh dear
        return null;
    }
  }

  // public static Command postScore(int level, Wrist wrist, Elevator elevator) {
  // return prepForScoring(level, wrist, elevator);
  // // }

  // public static Command prepForScoreAutoPath(
  //     int level, Wrist wrist, Elevator elevator, Drive drive) {
  //   return prepForScoring(level, wrist, elevator)
  //       .alongWith(new PathfindToClosestDepotCommand(drive));
  // }
}
