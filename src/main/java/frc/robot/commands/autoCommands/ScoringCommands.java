package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.commands.elevator.SetElevatorPresetCommand;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.WristAngle;

public class ScoringCommands {

  public static Command prepForScoringAuto(int level, Wrist wrist, Elevator elevator) {
    switch (level) { // bit of a misnomer here
      case 1:
        return new InstantCommand(() -> elevator.setTargetHeight(Constants.Elevator.STAGE_2_LEVEL))
            .alongWith(
                new InstantCommand(() -> wrist.setTargetAngle(WristAngle.STAGE2_ANGLE.getAngle())));
      case 2:
        return new InstantCommand(() -> elevator.setTargetHeight(Constants.Elevator.STAGE_3_LEVEL))
            .alongWith(
                new InstantCommand(() -> wrist.setTargetAngle(WristAngle.STAGE2_ANGLE.getAngle())));

      case 3:
        return new InstantCommand(() -> elevator.setTargetHeight(Constants.Elevator.STAGE_4_LEVEL))
            .alongWith(
                new InstantCommand(() -> wrist.setTargetAngle(WristAngle.STAGE2_ANGLE.getAngle())));

      default:
        return null;
    }
  }

  public static Command prepForScoring(int level, Wrist wrist, Elevator elevator) {
   
    
      switch (level) { // bit of a misnomer here
        case 1:
          return new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.STAGE_2_LEVEL)
              .alongWith(new SetWristTargetAngleCommand(wrist, WristAngle.STAGE2_ANGLE.getAngle()));

        case 2:
          return new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.STAGE_3_LEVEL)
              .alongWith(new SetWristTargetAngleCommand(wrist, WristAngle.STAGE2_ANGLE.getAngle()));

        case 3:
          return new SetElevatorPresetCommand(
                  elevator, wrist, Constants.Elevator.MAX_HEIGHT - 0.1) // just a bit less than max
              .alongWith(new SetWristTargetAngleCommand(wrist, WristAngle.STAGE2_ANGLE.getAngle()));

        default:
          return null;
      }
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

