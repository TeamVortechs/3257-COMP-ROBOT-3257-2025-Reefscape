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

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandautos.midStartOneAlgae;
import frc.robot.commands.autoCommands.DriveCommands;
import frc.robot.commands.autoCommands.ScoringCommands;
import frc.robot.commands.communication.ControllerVibrateCommand;
import frc.robot.commands.communication.TellCommand;
import frc.robot.commands.elevator.SetElevatorPresetCommand;
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
import frc.robot.subsystems.elevator.ElevatorModuleIO;
import frc.robot.subsystems.elevator.ElevatorModuleIOSimulation;
import frc.robot.subsystems.elevator.ElevatorModuleTalonFXIO;
// import frc.robot.subsystems.elevator.Elevator2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.WristAngle;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSimulation;
import frc.robot.subsystems.wrist.WristIOTalonFX;
// import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.util.simulation.MechanismSimulator;
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
  private final Wrist wrist;

  // DigitalInput limitSwitch =
  // new DigitalInput(20); // !!!!! FAKE CHANNEL! CHANGE WHEN PROPERLY IMPLEMENTED !!!!!!
  // private final Intake intake = new Intake(new IntakeIOTalonFX(), limitSwitch);
  private final Elevator elevator;

  private final MechanismSimulator sim;

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
  private final SendableChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> loggedAutoChooser;

  // pathconstraints for pathplanner paths
  private final PathConstraints pathConstraints =
      new PathConstraints(1, 0.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

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
                new VisionIO() {},
                new VisionIO() {}); // disable vision in match
        // new Vision(
        //     drive::addVisionMeasurement,
        //     // new VisionIOPhotonVision(
        //     //     VisionConstants.ARDUCAM_LEFT_NAME, VisionConstants.ROBOT_TO_ARDUCAM_LEFT),
        //     new VisionIOPhotonVision(
        //         VisionConstants.ARDUCAM_RIGHT_NAME, VisionConstants.ROBOT_TO_ARDUCAM_RIGHT));
        wrist =
            new Wrist(
                new WristIOTalonFX(
                    Constants.Arm.ARM_MOTOR_ID, Constants.Arm.ROLLER_MOTOR_ID, Constants.Arm.CANBUS
                    //   Constants.Arm.CANRANGE_ID
                    ));
        elevator =
            new Elevator(
                new ElevatorModuleTalonFXIO(
                    Constants.Elevator.MOTOR_LEFT_ID,
                    Constants.Elevator.MOTOR_RIGHT_ID,
                    Constants.Elevator.CANBUS),
                wrist);
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
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.ARDUCAM_LEFT_NAME,
        //             VisionConstants.ROBOT_TO_ARDUCAM_LEFT,
        //             drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.ARDUCAM_RIGHT_NAME,
        //             VisionConstants.ROBOT_TO_ARDUCAM_RIGHT,
        //             drive::getPose));
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        wrist = new Wrist(new WristIOSimulation());
        elevator = new Elevator(new ElevatorModuleIOSimulation(), wrist);
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
        wrist = new Wrist(new WristIO() {});
        elevator = new Elevator(new ElevatorModuleIO() {}, wrist);
        break;
    }

    sim = new MechanismSimulator(wrist, elevator);

    // registerNamedCommandsAuto();

    autoChooser = new SendableChooser<>();
    // autoChooser.addDefaultOption("Clear", getAutonomousCommand());
    autoChooser.setDefaultOption(
        "Blue Clear",
        DriveCommands.joystickDrive(drive, () -> 0.6, () -> 0, () -> 0).withTimeout(2));
    autoChooser.addOption(
        "Red Clear",
        DriveCommands.joystickDrive(drive, () -> -0.6, () -> 0, () -> 0).withTimeout(2));
    autoChooser.addOption("Mid Start, One Algae", new midStartOneAlgae(wrist, elevator, drive));

    // registerAutoChooser();

    // set up the logged version of the auto chooser
    loggedAutoChooser = new LoggedDashboardChooser<>("Auto Choices", autoChooser);

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

    /*
     * default commands (run when nothing else is running)
     */
    // driver controller's left stick controls lateral movement
    // right stick controls rotation
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    // constantly slowly intake to keep algae from falling out uness the wrist has a coral in it as
    // part of startup
    wrist.setDefaultCommand(
        new ConditionalCommand(
            new SetWristRollerSpeedCommand(wrist, Constants.Arm.ROLLER_HOLDING_POWER),
            new SetWristRollerSpeedCommand(wrist, 0),
            () -> !wrist.hasCoral()));

    /*
     * driver control binds
     */
    // L3 sets to upper-algae position
    controller.leftStick().onTrue(ScoringCommands.prepForScoring(2, wrist, elevator));
    // R3 sets to lower-algae position
    controller.rightStick().onTrue(ScoringCommands.prepForScoring(1, wrist, elevator));
    // L1/LB sets to ground-intake position while held if low enough; on release, sets arm back to
    // elevator-ready
    // position
    controller
        .leftBumper()
        .onTrue(ScoringCommands.prepForScoring(6, wrist, elevator))
        .onFalse(
            new InstantCommand(
                    () -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER), wrist)
                .andThen(
                    new SetWristTargetAngleCommand(wrist, () -> Constants.Arm.SCORING_ANGLE)
                        .onlyIf(
                            () ->
                                elevator.getCurrentHeight() <= Constants.Elevator.INTAKE_LEVEL_2)));
    // L2/LT intakes algae while held
    controller.leftTrigger().whileTrue(new RunCommand(() -> wrist.setRollerSpeed(0.4), wrist));
    // R1/RB sets to barge-scoring position
    controller.rightBumper().onTrue(ScoringCommands.prepForScoring(3, wrist, elevator));
    // R2/RB ejects algae while held, then sets to floor on release
    controller
        .rightTrigger()
        .whileTrue(new SetWristRollerSpeedCommand(wrist, -1))
        .onFalse(ScoringCommands.prepForScoring(4, wrist, elevator));
    // B sets elevator to minimum height
    controller.b().onTrue(ScoringCommands.prepForScoring(4, wrist, elevator));
    // X sets the arm to processor-scoring position
    controller.x().onTrue(ScoringCommands.prepForScoring(5, wrist, elevator));
    // Y fully retracts the arm (sets the arm's angle to 0)
    controller
        .y()
        .onTrue(
            new InstantCommand(
                    () -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER), wrist)
                .andThen(new SetWristTargetAngleCommand(wrist, () -> 0)));
    // start resets arm and elevator encoders
    controller
        .start()
        .onTrue(
            new InstantCommand(() -> elevator.resetEncoders())
                .ignoringDisable(true)
                .alongWith(new InstantCommand(() -> wrist.resetWristEncoder()))
                .ignoringDisable(true));
    // dpad up inverts the gyro
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
    // dpad down resets the gyro
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    /*
     * operator control binds
     */
    // L1/LB sets to lower-algae position
    operatorController.leftBumper().whileTrue(ScoringCommands.prepForScoring(2, wrist, elevator));
    // R1/RB sets to upper-algae position
    operatorController.rightBumper().whileTrue(ScoringCommands.prepForScoring(3, wrist, elevator));
    // L2/LT sets elevator to minimum height
    operatorController.leftTrigger().whileTrue(ScoringCommands.prepForScoring(4, wrist, elevator));
    // R2/RT intakes algae
    operatorController.rightTrigger().whileTrue(new SetWristRollerSpeedCommand(wrist, 0.6));
    // (DEPRECATED) A tells the arm to use PID to control itself
    operatorController
        .a()
        .onTrue(new SetWristTargetAngleCommand(wrist, () -> wrist.getTargetAngle()));
    // B sets the arm outwards manually
    operatorController
        .b()
        .whileTrue(new SetWristTargetAngleCommand(wrist, () -> WristAngle.STAGE2_ANGLE.getAngle()));
    // X fully retracts the arm
    operatorController
        .x()
        .whileTrue(new SetWristTargetAngleCommand(wrist, () -> WristAngle.INTAKE_ANGLE.getAngle()));
    // Y sets the arm to ground intake position
    operatorController
        .y()
        .whileTrue(
            new InstantCommand(
                    () -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER), wrist)
                .andThen(
                    new SetWristTargetAngleCommand(
                        wrist, () -> Constants.Arm.GROUND_INTAKE_ANGLE)));
    // dpad up manually moves arm outwards
    operatorController.povUp().whileTrue(new ManualSetWristSpeedCommand(wrist, () -> 0.15));
    // dpad down manually moves arm inwards
    operatorController.povDown().whileTrue(new ManualSetWristSpeedCommand(wrist, () -> -0.1));
  } // end configure bindings

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return loggedAutoChooser.get();
    // try {
    //   final var path = PathPlannerPath.fromPathFile("Clear");
    //   return AutoBuilder.followPath(path);
    // } catch (Exception ignored) {

    // }
    // return null;
    // if (DriverStation.getAlliance().get()
    //     == DriverStation.Alliance.Red) { // If red, go in reverse (forwards on red side)
    //   return DriveCommands.joystickDrive(drive, () -> -0.6, () -> 0, () -> 0)
    //       .andThen(Commands.waitSeconds(2))
    //       .andThen(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0));
    // } else { // If not, go forwards (forwards on blue side)
    //   return DriveCommands.joystickDrive(drive, () -> 0.6, () -> 0, () -> 0)
    //       .andThen(Commands.waitSeconds(2))
    //       .andThen(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0));
    // }
    // return DriveCommands.joystickDrive(drive, () -> 0.6, () -> 0, () -> 0)
    //     .andThen(Commands.waitSeconds(2))
    //     .andThen(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0));
  }

  // registers pathplanner's named commands
  private void registerNamedCommandsAuto() {
    // controls wether or not the robot actually does the commands or just prints out that it's
    // doing the commands

    // stuff to check wether or not it was a sim
    boolean isReal = true;
    // if (Constants.currentMode == Mode.SIM) isReal = false;

    // comm
    addNamedCommand("intakeStage1", ScoringCommands.intakeAuto(1, wrist, elevator), isReal);

    addNamedCommand("intakeStage2", ScoringCommands.intakeAuto(2, wrist, elevator), isReal);

    addNamedCommand(
        "prepIntakeStage1", ScoringCommands.prepForIntakeAuto(1, wrist, elevator), isReal);
    addNamedCommand(
        "prepIntakeStage2", ScoringCommands.prepForIntakeAuto(2, wrist, elevator), isReal);

    addNamedCommand("score", ScoringCommands.scoreAuto(wrist, elevator), isReal);

    addNamedCommand(
        "mechanismBack",
        new InstantCommand(() -> wrist.setRollerSpeed(Constants.Arm.ROLLER_HOLDING_POWER))
            .andThen(new SetElevatorPresetCommand(elevator, 0))
            .andThen(new SetWristTargetAngleCommand(wrist, () -> 0)),
        isReal);

    // unbounded this for now bc we don't know
    addNamedCommand("coralScore", ScoringCommands.coralScoreAuto(wrist), isReal);

    // addNamedCommand("coralScore", ScoringCommands.coralScoreAuto(wrist), isReal);
  }

  // function to add named commands because we need to add is an an event too and not just as a
  // command. This also handles simulation logging
  public void addNamedCommand(String commandName, Command command, boolean isReal) {

    if (isReal) {
      NamedCommands.registerCommand(
          commandName, command.andThen(new TellCommand("just ran " + commandName)));
      //   new EventTrigger(commandName).onTrue(command);
    } else {
      // registers the named commands to print something out instead of actually running anything
      NamedCommands.registerCommand(
          commandName,
          new TellCommand(commandName + " auto command")
              .andThen(
                  new ControllerVibrateCommand(1, controller).withDeadline(new WaitCommand(0.2)))
              .alongWith(command));

      //   new EventTrigger(commandName)
      //       .onTrue(
      //           new TellCommand(commandName + " auto event trigger command")
      //               .andThen(
      //                   new ControllerVibrateCommand(1, controller)
      //                       .withDeadline(new WaitCommand(0.2)))
      //               .andThen(new WaitCommand(0.3)));
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

  public Wrist getWrist() {
    return wrist;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public MechanismSimulator getSim() {
    return sim;
  }
}
