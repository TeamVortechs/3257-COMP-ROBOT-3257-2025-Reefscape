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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.autoCommands.DriveCommands;
import frc.robot.commands.autoCommands.IntakingCommands;
import frc.robot.commands.autoCommands.ScoringCommands;
import frc.robot.commands.communication.ControllerVibrateCommand;
import frc.robot.commands.communication.TellCommand;
import frc.robot.commands.elevator.ManualElevatorCommand;
import frc.robot.commands.elevator.SetElevatorPresetCommand;
import frc.robot.commands.pathfindingCommands.PathfindToClosestDepotCommand;
import frc.robot.commands.pathfindingCommands.PathfindingCommandCancel;
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
              Constants.Arm.ARM_MOTOR_ID,
              Constants.Arm.ROLLER_MOTOR_ID,
              Constants.Arm.CANBUS,
              Constants.Arm.CANRANGE_ID));

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

    // controller
    //     .start()
    //     .onTrue(
    //         new InstantCommand(() -> elevator.resetEncoders())
    //             .ignoringDisable(true)
    //             .alongWith(new InstantCommand(() -> wrist.resetWristEncoder()))
    //             .ignoringDisable(true));

    // eject note as long as button as help
    // controller.rightBumper().whileTrue(new SetWristRollerSpeedCommand(wrist, -0.3));

    // moves elevator and wrist to the scoring positions level 2 after the right button is tapped
    controller.leftTrigger().whileTrue(ScoringCommands.prepForScoring(1, wrist, elevator));

    // moves elevator and wrist to the scoring positions level 2 after the right button is tapped
    controller.leftBumper().whileTrue(ScoringCommands.prepForScoring(2, wrist, elevator));

    // moves elevator and wrist to scoring position for level 3
    controller.rightBumper().whileTrue(ScoringCommands.prepForScoring(3, wrist, elevator));

    // intakes then vibrates controlller when in position and has coral
    // y shoots coral out
    controller.a().whileTrue(new SetWristRollerSpeedCommand(wrist, -0.75));

    // left bumper sets the wrist outwards manually
    controller
        .b()
        .whileTrue(new SetWristTargetAngleCommand(wrist, WristAngle.STAGE2_ANGLE.getAngle()));

    // right trigger sets elevator height back down to 0
    controller
        .rightTrigger()
        .whileTrue(
            // new InstantCommand(() ->
            // elevator.setTargetHeight(Constants.Elevator.INTAKE_HEIGHT)));
            IntakingCommands.intakeCommand(wrist, elevator)
                // vibrates the controller for half a second after intake
                .andThen(
                    Commands.deadline(
                        new WaitCommand(0.5), new ControllerVibrateCommand(0.7, controller))));

    // old elevator default command
    elevator.setDefaultCommand(
        new WaitCommand(2)
            .andThen(
                new SetElevatorPresetCommand(elevator, wrist, 0)
                    .unless(() -> wrist.isCanCloserThan(0.1))));

    // if there is a note move the wrist to scoring position. If there is not a note move the wrist
    // back to intake position when the elevator is on the floor
    wrist.setDefaultCommand(
        new ConditionalCommand(
            // if there is a note move the wrist angle to the shooting angle
            (new SetWristTargetAngleCommand(wrist, Constants.Arm.WRIST_STAGE_2_ANGLE)),
            // if there is not a note move the wrist to the target angle 0
            new SetWristTargetAngleCommand(wrist, 0)
                // unless the elevator is not on the floor
                .unless(() -> !elevator.isOnFloor()),
            // controller of the conditional
            () -> wrist.isCanCloserThan(0.1)));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.povLeft().whileTrue(new SetWristRollerSpeedCommand(wrist, 0.05));

    controller.x().whileTrue(new PathfindToClosestDepotCommand(drive, false));
    controller.x().onFalse(new PathfindingCommandCancel(drive));

    controller.y().whileTrue(new PathfindToClosestDepotCommand(drive, true));
    controller.y().onFalse(new PathfindingCommandCancel(drive));

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

    // operator controls
    operatorController.leftTrigger().whileTrue(ScoringCommands.prepForScoring(1, wrist, elevator));

    // moves elevator and wrist to the scoring positions level 2 after the right button is tapped
    operatorController.leftBumper().whileTrue(ScoringCommands.prepForScoring(2, wrist, elevator));

    // moves elevator and wrist to scoring position for level 3
    operatorController.rightBumper().whileTrue(ScoringCommands.prepForScoring(3, wrist, elevator));

    operatorController
        .start()
        .onTrue(
            new InstantCommand(() -> elevator.resetEncoders())
                .ignoringDisable(true)
                .alongWith(new InstantCommand(() -> wrist.resetWristEncoder()))
                .ignoringDisable(true));
  } // end configure bindings

  public void addExperimentalOperatorCommands() {
    operatorController.povUp().whileTrue(new ManualSetWristSpeedCommand(wrist, () -> 0.1));
    operatorController.povDown().whileTrue(new ManualSetWristSpeedCommand(wrist, () -> -0.1));

    operatorController.povRight().whileTrue(new ManualElevatorCommand(elevator, () -> 0.1));
    operatorController.povLeft().whileTrue(new ManualElevatorCommand(elevator, () -> -0.1));

    operatorController.x().whileTrue(new SetWristRollerSpeedCommand(wrist, 0.5));
    operatorController.y().whileTrue(new SetWristRollerSpeedCommand(wrist, -0.5));

    operatorController.b().whileTrue(IntakingCommands.intakeCommand(wrist, elevator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // adds a disturbance to the robot for a future limelight. This was tested in sim
  public void bindDisturbanceCommand() {
    // add a free disturbance when pressing the y button to test vision
    var disturbance =
        new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(0.17 * 2 * Math.PI));
    controller
        .back()
        .onTrue(
            Commands.runOnce(() -> drive.setPose(drive.getPose().plus(disturbance)))
                .ignoringDisable(true));
  }

  // registers pathplanner's named commands. This was tested in sim but on the robot yet.
  private void registerNamedCommandsAuto() {
    // controls wether or not the robot actually does the commands or just prints out that it's
    // doing the commands

    // stuff to check wether or not it was a sim
    boolean isReal = true;
    // if (Constants.currentMode == Mode.SIM) isReal = false;

    // comm
    addNamedCommand("intake prep", IntakingCommands.prepForIntakeCommand(wrist, elevator), isReal);
    addNamedCommand("intake", IntakingCommands.intakeCommand(wrist, elevator), isReal);
    addNamedCommand("prepStage1", ScoringCommands.prepForScoring(1, wrist, elevator), isReal);
    addNamedCommand("prepStage2", ScoringCommands.prepForScoring(2, wrist, elevator), isReal);
    addNamedCommand(
        "Scoring",
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
