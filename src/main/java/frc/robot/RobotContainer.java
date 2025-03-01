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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autoCommands.DriveCommands;
import frc.robot.commands.autoCommands.IntakingCommands;
import frc.robot.commands.autoCommands.ScoringCommands;
import frc.robot.commands.communication.TellCommand;
import frc.robot.commands.wrist.ManualSetWristSpeedCommand;
import frc.robot.commands.wrist.SetWristRollerSpeedCommand;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
// import frc.robot.commands.SetWristRollerSpeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorModuleTalonFXIO;
// import frc.robot.subsystems.elevator.Elevator2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.WristAngle;
import frc.robot.subsystems.wrist.WristIOTalonFX;
// import frc.robot.subsystems.wrist.WristIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final Vision vision;

  // physical subsystems
  private final Wrist wrist =
      new Wrist(
          new WristIOTalonFX(
              Constants.Arm.ARM_MOTOR_ID, Constants.Arm.ROLLER_MOTOR_ID, Constants.Arm.CANBUS
              //   Constants.Arm.CANRANGE_ID
              ));

  // DigitalInput limitSwitch =
  // new DigitalInput(20); // !!!!! FAKE CHANNEL! CHANGE WHEN PROPERLY IMPLEMENTED !!!!!!
  // private final Intake intake = new Intake(new IntakeIOTalonFX(), limitSwitch);
  private final Elevator elevator =
      new Elevator(
          new ElevatorModuleTalonFXIO(
              Constants.Elevator.MOTOR_LEFT_ID,
              Constants.Elevator.MOTOR_RIGHT_ID,
              Constants.Elevator.CANBUS),
          wrist);
  //   private final Elevator2 elevator2 =
  //       new Elevator2(
  //           new ElevatorModuleTalonFXIO(
  //               Constants.ELEVATOR_MOTOR_LEFT_ID,
  //               Constants.ELEVATOR_MOTOR_RIGHT_ID,
  //               Constants.ELEVATOR_CANBUS));

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // pathconstraints for pathplanner paths
  private final PathConstraints pathConstraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                // new VisionIOPhotonVision(
                //     VisionConstants.ARDUCAM_LEFT_NAME, VisionConstants.ROBOT_TO_ARDUCAM_LEFT),
                new VisionIOPhotonVision(
                    VisionConstants.ARDUCAM_RIGHT_NAME, VisionConstants.ROBOT_TO_ARDUCAM_RIGHT));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.ARDUCAM_LEFT_NAME,
                    VisionConstants.ROBOT_TO_ARDUCAM_LEFT,
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.ARDUCAM_RIGHT_NAME,
                    VisionConstants.ROBOT_TO_ARDUCAM_RIGHT,
                    drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    registerNamedCommandsAuto();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // registerAutoChooser();
    // configure the autonomous named commands

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // controller.leftTrigger().whileTrue(new PathfindToClosestDepotCommand(drive));
    // Default command, normal field-relative drive

    // controller.start().whileTrue(new WristSetPosCommand(wrist, 0.25));
    //  controller.back().whileTrue(new WristSetPosCommand(wrist, -0.25));
    // controller2.leftBumper().whileTrue(new IntakeSpeedCommand(intake, 0.75, limitSwitch));
    // controller.b().whileTrue(elevator2.runCurrentZeroing());
    // controller.rightBumper().whileTrue(new SetElevatorPower(elevator2, 0.1));
    // controller.leftBumper().whileTrue(new SetElevatorPower(elevator2, -0.1));
    // controller
    //     .rightBumper()
    //     .whileTrue(new SetElevatorCommand(ElevatorLevel.FIRST_LEVEL, elevator2));
    // controller
    //     .rightTrigger()
    //     .whileTrue(new SetElevatorCommand(ElevatorLevel.SECOND_LEVEL, elevator2));
    // controller
    //     .leftTrigger()
    //     .whileTrue(new SetElevatorCommand(ElevatorLevel.THIRD_LEVEL, elevator2));

    // controller
    //     .a()
    //     .whileTrue(
    //         new TellCommand()
    //             .andThen(
    //                 new SetWristRollerSpeed(wrist, -0.01)
    //                     .unless(() -> wrist.isCanCloserThan(0.1))));

    // wrist.setDefaultCommand(
    //     new ConditionalCommand(
    //         new SetWristTargetAngleCommand(wrist, WristAngle.STAGE1_ANGLE.getAngle()),
    //         new SetWristTargetAngleCommand(wrist, 0),
    //         () - !wrist.isCanCloserThan(0.1)));
    /* */
    // resets encoders. THIS WILL BREAK THE ROBOT
    controller
        .start()
        .onTrue(
            new InstantCommand(() -> elevator.resetEncoders())
                .ignoringDisable(true)
                .alongWith(new InstantCommand(() -> wrist.resetWristEncoder()))
                .ignoringDisable(true));

    // eject note as long as button as help
    // controller.rightBumper().whileTrue(new SetWristRollerSpeedCommand(wrist, -0.3));

    // moves elevator and wrist to the scoring positions level 2 after the right button is tapped
    controller.leftTrigger().whileTrue(ScoringCommands.prepForScoring(1, wrist, elevator));

    // moves elevator and wrist to the scoring positions level 2 after the right button is tapped
    controller.leftBumper().whileTrue(ScoringCommands.prepForScoring(2, wrist, elevator));

    // moves elevator and wrist to scoring position for level 3
    controller.rightBumper().whileTrue(ScoringCommands.prepForScoring(3, wrist, elevator));

    operatorController.leftTrigger().whileTrue(ScoringCommands.prepForScoring(1, wrist, elevator));

    // moves elevator and wrist to the scoring positions level 2 after the right button is tapped
    operatorController.leftBumper().whileTrue(ScoringCommands.prepForScoring(2, wrist, elevator));

    // moves elevator and wrist to scoring position for level 3
    operatorController.rightBumper().whileTrue(ScoringCommands.prepForScoring(3, wrist, elevator));

    operatorController.y().whileTrue(ScoringCommands.prepForScoring(4, wrist, elevator));

    operatorController.rightTrigger().whileTrue(new SetWristRollerSpeedCommand(wrist, 0.6));
    // new InstantCommand(() ->
    // elevator.setTargetHeight(Constants.Elevator.INTAKE_HEIGHT)));
    // IntakingCommands.intakeCommand(wrist, elevator)
    // // vibrates the controller for half a second after intake
    // .andThen(
    //     Commands.deadline(
    //         new WaitCommand(0.5), new ControllerVibrateCommand(0.7, controller)))
    // );

    // intakes then vibrates controlller when in position and has coral
    // driver A shoots algae
    controller.a().whileTrue(new SetWristRollerSpeedCommand(wrist, -1));

    // left bumper sets the wrist outwards manually
    operatorController
        .b()
        .whileTrue(new SetWristTargetAngleCommand(wrist, () -> WristAngle.STAGE2_ANGLE.getAngle()));

    operatorController
        .x()
        .whileTrue(new SetWristTargetAngleCommand(wrist, () -> WristAngle.INTAKE_ANGLE.getAngle()));

    // controller.leftTrigger().whileTrue(new ManualElevatorCommand(elevator, () -> -0.2));
    // controller.rightTrigger().whileTrue(new ManualElevatorCommand(elevator, () -> 0.2));

    // left trigger sets height to Stage 2
    /*controller
        .leftTrigger()
        .whileTrue(
            new InstantCommand(() -> elevator.setTargetHeight(Constants.Elevator.STAGE_2_LEVEL)));
    // right trigger sets height to Stage 3
    controller
        .rightTrigger()
        .whileTrue(
            new InstantCommand(() -> elevator.setTargetHeight(Constants.Elevator.STAGE_3_LEVEL)));
            /* */
    // driver right trigger intakes algae
    controller.rightTrigger().whileTrue(new SetWristRollerSpeedCommand(wrist, 0.6));

    /*controller
            .rightTrigger()
            .whileTrue(
                // new InstantCommand(() ->
                // elevator.setTargetHeight(Constants.Elevator.INTAKE_HEIGHT)));
                IntakingCommands.intakeCommand(wrist, elevator)
                    // vibrates the controller for half a second after intake
                    .andThen(
                        Commands.deadline(
                            new WaitCommand(0.5), new ControllerVibrateCommand(0.7, controller))));
    /* */
    // old elevator default command
    // elevator.setDefaultCommand(
    //     new WaitCommand(2) // wait two seconds, then
    //         .andThen(
    //             new SetElevatorPresetCommand(elevator, wrist, 0) // set elevator to minimum
    // height
    //                 .unless(() -> wrist.isCanCloserThan(0.1)))); // unless there is a coral

    // if there is no note move the elevator down to zero. If there is a note move elevator to first
    // level if it is currently below first level
    // elevator.setDefaultCommand(
    //     new ConditionalCommand(
    //         // set the elevator to move up to stage 1 if it's below and has the coral(that way
    // cycle
    //         // time is increased if they forgot to do it)
    //         new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.STAGE_2_LEVEL)
    //             .unless(() -> elevator.getTargetHeight() > Constants.Elevator.STAGE_2_LEVEL),
    //         // sets the the elevator to go zero if it doesn't have a coral
    //         new SetElevatorPresetCommand(elevator, wrist, 0),
    //         // conditional that controls the elevator
    //         () -> wrist.isCanCloserThan(0.1)));

    // if there is a note move the wrist to scoring position. If there is not a note move the wrist
    // back to intake position when the elevator is on the floor
    // wrist.setDefaultCommand(
    //     new ConditionalCommand(
    //         // if there is a note move the wrist angle to the shooting angle
    //         (new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.WRIST_STAGE_2_ANGLE)),
    //         // if there is not a note move the wrist to the target angle 0
    //         new SetWristTargetAngleCommand(wrist, () -> 0)
    //             // unless the elevator is not on the floor
    //             .unless(() -> !elevator.isOnFloor()),
    //         // controller of the conditional
    //         () -> wrist.isCanCloserThan(0.1)));
    wrist.setDefaultCommand(new SetWristRollerSpeedCommand(wrist, 0.2));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.povLeft().whileTrue(new SetWristRollerSpeedCommand(wrist, 0.05));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d( )));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // controller.x().whileTrue(new PathfindToClosestDepotCommand(drive, false));
    // controller.x().onFalse(new PathfindingCommandCancel(drive));

    // controller.y().whileTrue(new PathfindToClosestDepotCommand(drive, true));
    // controller.y().onFalse(new PathfindingCommandCancel(drive));

    operatorController.povDown().whileTrue(new ManualSetWristSpeedCommand(wrist, () -> -0.1));
    operatorController.povUp().whileTrue(new ManualSetWristSpeedCommand(wrist, () -> 0.15));
    operatorController
        .a()
        .onTrue(new SetWristTargetAngleCommand(wrist, () -> wrist.getTargetAngle()));

    // controller.x().whileTrue(new PathfindToClosestDepotCommand(drive, false));
    // controller.x().onFalse(new PathfindingCommandCancel(drive));

    // controller.y().whileTrue(new PathfindToClosestDepotCommand(drive, true));
    // controller.y().onFalse(new PathfindingCommandCancel(drive));

    // controller
    //     .leftTrigger()
    //     .whileTrue(
    //         PathfindingCommands.pathfindToDepotCommand(
    //             PathfindingCommands.getClosestDepotPath(drive.getPose())));

    // // Reset gyro to 0° when B button is pressed
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(), Rotation2d.fromDegrees(180))),
                    drive)
                .ignoringDisable(true));

    // add a free disturbance when pressing the y button to test vision
    // var disturbance =
    //     new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(0.17 * 2 * Math.PI));
    // controller
    //     .y()
    //     .onTrue(
    //         Commands.runOnce(() -> drive.setPose(drive.getPose().plus(disturbance)))
    //             .ignoringDisable(true));
  } // end configure bindings

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // registers pathplanner's named commands
  private void registerNamedCommandsAuto() {
    // controls wether or not the robot actually does the commands or just prints out that it's
    // doing the commands

    // stuff to check wether or not it was a sim
    boolean isReal = false;
    // if (Constants.currentMode == Mode.SIM) isReal = false;

    // comm
    addNamedCommand("intakeStage2", IntakingCommands.intakeCommandAuto(wrist, elevator, 2), isReal);
    addNamedCommand("intakeStage1", IntakingCommands.intakeCommandAuto(wrist, elevator, 1), isReal);
    addNamedCommand(
        "score",
        new WaitCommand(0.2).deadlineFor(new SetWristRollerSpeedCommand(wrist, -0.4)),
        isReal);
  }

  // function to add named commands because we need to add is an an event too and not just as a
  // command. This also handles simulation logging
  public void addNamedCommand(String commandName, Command command, boolean isReal) {

    if (isReal) {
      NamedCommands.registerCommand(commandName, command);
      new EventTrigger(commandName).onTrue(command);
    } else {
      // registers the named commands to print something out instead of actually running anything
      NamedCommands.registerCommand(commandName, new TellCommand(commandName + " auto command"));
      new EventTrigger(commandName)
          .onTrue(new TellCommand(commandName + " auto event trigger command"));
    }
  }

  //   public void sendVisionMeasurement() {
  //     // Correct pose estimate with vision measurements
  //     var visionEst = vision.getEstimatedGlobalPose();
  //     visionEst.ifPresent(
  //         est -> {
  //           // Change our trust in the measurement based on the tags we can see
  //           var estStdDevs = vision.getEstimationStdDevs();

  //           drive.addVisionMeasurement(
  //               est.estimatedPose.toPose2d(),
  //               est.timestampSeconds,
  //               estStdDevs); // !!! note: the standard deviation in the constants has to be
  // tweaked
  //         });
  //   }

  // intended for testing usage only
  // puts sendables on shuffleboard
  public void putPositionData() {
    SmartDashboard.putNumber("x position:", drive.getPose().getX());
    SmartDashboard.putNumber("y position:", drive.getPose().getY());
    SmartDashboard.putNumber("current rotation:", drive.getPose().getRotation().getDegrees());
  }
}
