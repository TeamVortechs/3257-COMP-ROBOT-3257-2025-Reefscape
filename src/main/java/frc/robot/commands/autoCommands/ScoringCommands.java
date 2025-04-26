package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.elevator.SetElevatorPresetCommand;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class ScoringCommands {

  public static Command coralScoreAuto(Wrist wrist) {
    return new InstantCommand(() -> wrist.setRollerSpeed(0))
        .andThen(
            SetWristTargetAngleCommand.withConsistentEnd(
                wrist, () -> Constants.Arm.WRIST_CORAL_SCORE))
        .andThen(new InstantCommand(() -> wrist.setRollerSpeed(-0.2)))
        .andThen(new WaitCommand(5))
        .andThen(new InstantCommand(() -> wrist.setHasCoral(false)));
  }

  /*
  old code:
        case 1: // low reef algae
        return new InstantCommand(() -> wrist.setHasCoral(false))
            .andThen(new InstantCommand(() -> wrist.setRollerSpeed(0.2), wrist))
            .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.REEF_INTAKE_ANGLE))
            .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
            .andThen(
                new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.INTAKE_LEVEL_1))
            .andThen(new WaitUntilCommand(() -> wrist.isOnTarget()))
            .andThen(new WaitCommand(1));

      case 2: // high reef algae
        return new InstantCommand(() -> wrist.setHasCoral(false))
            .andThen(new InstantCommand(() -> wrist.setRollerSpeed(0.2), wrist))
            .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.REEF_INTAKE_ANGLE))
            .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
            .andThen(
                new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.INTAKE_LEVEL_2))
            .andThen(new WaitUntilCommand(() -> wrist.isOnTarget()))
            .andThen(new WaitCommand(1));
   */

  public static Command intakeAuto(int level, Wrist wrist, Elevator elevator) {
    switch (level) { // bit of a misnomer here
      case 1: // low reef algae
        // return new InstantCommand(() -> wrist.setHasCoral(false))
        //     .andThen(new RunCommand(() -> wrist.setRollerSpeed(0.6)))
        //     .withDeadline(
        //         SetWristTargetAngleCommand.withConsistentEnd(
        //                 wrist, () -> Constants.Arm.REEF_INTAKE_ANGLE)
        //             .andThen(
        //                 SetElevatorPresetCommand.withEndCondition(
        //                     elevator, Constants.Elevator.INTAKE_LEVEL_1))
        //             .andThen(new WaitCommand(0.1)));
        return new InstantCommand(() -> wrist.setRollerSpeed(0.4), wrist)
            .andThen(new WaitCommand(0.2))
            .andThen(
                new InstantCommand(() -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER)));

      case 2: // high reef algae
        return new InstantCommand(() -> wrist.setHasCoral(false))
            .andThen(new RunCommand(() -> wrist.setRollerSpeed(0.6)))
            .withDeadline(
                SetWristTargetAngleCommand.withConsistentEnd(
                        wrist, () -> Constants.Arm.REEF_INTAKE_ANGLE)
                    .andThen(
                        SetElevatorPresetCommand.withEndCondition(
                            elevator, Constants.Elevator.INTAKE_LEVEL_1))
                    .andThen(new WaitCommand(0.1)));

      default:
        return new InstantCommand();
    }
  }

  public static Command prepForIntakeAuto(int level, Wrist wrist, Elevator elevator) {
    switch (level) { // bit of a misnomer here
      case 1: // low reef algae
        return new InstantCommand(() -> wrist.setHasCoral(false))
            .andThen(
                SetWristTargetAngleCommand.withConsistentEnd(
                    wrist, () -> Constants.Arm.REEF_INTAKE_ANGLE))
            .andThen(
                SetElevatorPresetCommand.withEndCondition(
                    elevator, Constants.Elevator.INTAKE_LEVEL_1));

      case 2: // high reef algae
        return new InstantCommand(() -> wrist.setHasCoral(false))
            .andThen(
                SetWristTargetAngleCommand.withConsistentEnd(
                    wrist, () -> Constants.Arm.REEF_INTAKE_ANGLE))
            .andThen(
                SetElevatorPresetCommand.withEndCondition(
                    elevator, Constants.Elevator.INTAKE_LEVEL_2));

      default:
        return new InstantCommand();
    }
  }

  public static Command scoreAuto(Wrist wrist, Elevator elevator) {
    return new InstantCommand(() -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER), wrist)
        .andThen(
            new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.ELEVATOR_CLEARANCE_ANGLE))
        .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
        .andThen(
            SetElevatorPresetCommand.withEndCondition(elevator, Constants.Elevator.BARGE_LEVEL))
        .andThen(SetWristTargetAngleCommand.withConsistentEnd(wrist, () -> 0))
        .andThen(new InstantCommand(() -> wrist.setRollerSpeed(-1)).andThen(new WaitCommand(1)));
  }

  public static Command prepForScoring(int level, Wrist wrist, Elevator elevator) {
    switch (level) { // bit of a misnomer here
      case 1: // low reef algae
        return new InstantCommand(
                () -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER), wrist)
            .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.REEF_INTAKE_ANGLE))
            .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
            .andThen(new SetElevatorPresetCommand(elevator, Constants.Elevator.INTAKE_LEVEL_1));
        // .andThen(
        //     new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.STAGE_2_LEVEL)
        //         .alongWith(
        //             new SetWristTargetAngleCommand(
        //                 wrist, () -> WristAngle.STAGE2_ANGLE.getAngle())));

      case 2: // high reef algae
        return new InstantCommand(
                () -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER), wrist)
            .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.REEF_INTAKE_ANGLE))
            .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
            .andThen(new SetElevatorPresetCommand(elevator, Constants.Elevator.INTAKE_LEVEL_2));
        // return new InstantCommand(() -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER),
        // wrist)
        //     .andThen(
        //         new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.STAGE_3_LEVEL)
        //             .alongWith(
        //                 new SetWristTargetAngleCommand(
        //                     wrist, () -> WristAngle.STAGE2_ANGLE.getAngle())));

      case 3: // scoring position
        return new InstantCommand(
                () -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER), wrist)
            .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.SCORING_ANGLE))
            .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
            .andThen(new SetElevatorPresetCommand(elevator, Constants.Elevator.BARGE_LEVEL))
            .andThen(
                new WaitUntilCommand(
                    () -> elevator.getCurrentHeight() > Constants.Elevator.INTAKE_LEVEL_2))
            .andThen(new SetWristTargetAngleCommand(wrist, () -> 0));
        // return new InstantCommand(() -> wrist.setRollerSpeed(0.2), wrist)
        //     .andThen(
        //         new SetElevatorPresetCommand(
        //                 elevator,
        //                 wrist,
        //                 Constants.Elevator.MAX_HEIGHT - 0.1) // just a bit less than max
        //             .alongWith(
        //                 new SetWristTargetAngleCommand(
        //                     wrist, () -> Constants.Arm.WRIST_STAGE_4_ANGLE)));
      case 4: // elevator ready state
        return new InstantCommand(
                () -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER), wrist)
            .andThen(new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.SCORING_ANGLE))
            .andThen(new WaitUntilCommand(() -> wrist.isClearFromElevator()))
            .andThen(new SetElevatorPresetCommand(elevator, Constants.Elevator.MIN_HEIGHT));
        // return new InstantCommand(() -> wrist.setRollerSpeed(0.2), wrist)
        //     .andThen(
        //         new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.MIN_HEIGHT)
        //             .alongWith(
        //                 new SetWristTargetAngleCommand(
        //                     wrist, () -> WristAngle.ALGAE_GROUND_INTAKE.getAngle())));

      case 5: // processor position
        return new InstantCommand(
                () -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER),
                wrist) // keep wrist auto intaking
            .andThen(
                new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.SCORING_ANGLE)
                    .unless(
                        () ->
                            elevator.getCurrentHeight() <= Constants.Elevator.INTAKE_LEVEL_2
                                && wrist.getAngleRotations()
                                    >= Constants.Arm
                                        .SCORING_ANGLE)) // if not below low algae position, set
            // wrist
            // only to clear position
            .andThen(
                new WaitUntilCommand(() -> wrist.isClearFromElevator())) // wait until it's clear
            .andThen(
                new SetElevatorPresetCommand(
                        elevator, Constants.Elevator.INTAKE_LEVEL_1) // bring elevator down
                    .andThen(
                        new WaitUntilCommand(
                            () ->
                                elevator.getCurrentHeight()
                                    <= Constants.Elevator
                                        .INTAKE_LEVEL_2))) // wait until the elevator's below intake
            // level 1 position
            .andThen(
                new SetWristTargetAngleCommand(
                    wrist, () -> Constants.Arm.PROCESSOR_ANGLE)); // bring the arm out

      case 6: // ground intake position
        return new InstantCommand(
                () -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER),
                wrist) // keep wrist auto intaking
            .andThen(
                new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.SCORING_ANGLE)
                    .unless(
                        () ->
                            elevator.getCurrentHeight() <= Constants.Elevator.INTAKE_LEVEL_2
                                && wrist.getAngleRotations()
                                    >= Constants.Arm
                                        .SCORING_ANGLE)) // if not below low algae position, set
            // wrist
            // only to clear position
            .andThen(
                new WaitUntilCommand(() -> wrist.isClearFromElevator())) // wait until it's clear
            .andThen(
                new SetElevatorPresetCommand(
                        elevator, Constants.Elevator.MIN_HEIGHT) // bring elevator down
                    .andThen(
                        new WaitUntilCommand(
                            () ->
                                elevator.getCurrentHeight()
                                    <= Constants.Elevator
                                        .INTAKE_LEVEL_1))) // wait until the elevator's below intake
            // level 1 position
            .andThen(
                new SetWristTargetAngleCommand(
                    wrist, () -> Constants.Arm.GROUND_INTAKE_ANGLE)); // bring the arm out
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
