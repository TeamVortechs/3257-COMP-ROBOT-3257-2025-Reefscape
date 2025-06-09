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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.communication.ControllerVibrateCommand;
import frc.robot.commands.communication.TellCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmSimulationIO;
import frc.robot.subsystems.canrange.RangeFinder;
import frc.robot.subsystems.canrange.RangeFinderIO;
import frc.robot.subsystems.canrange.RangeFinderSimulationIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorSimulationIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSimulation;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.FieldMovement.VortechsUtil;
import frc.robot.util.FieldMovement.pathfindingUtil.PathfinderVortechs;
import frc.robot.util.FieldMovement.pathfindingUtil.VortechsClosestPoseSupplier;
import java.util.ArrayList;
import java.util.List;
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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> loggedAutoChooser;

  private final Arm arm;
  private final Elevator elevator;

  private final Intake intake;
  private final RangeFinder canRange;

  private final PathfinderVortechs pathfinderVortechs;

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

        arm = new Arm(new ArmIO() {});
        elevator = new Elevator(new ElevatorIO() {}, arm);

        canRange = new RangeFinder(new RangeFinderIO() {});
        intake = new Intake(new IntakeIO() {}, canRange);
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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        arm = new Arm(new ArmSimulationIO());
        elevator = new Elevator(new ElevatorSimulationIO() {}, arm);

        canRange = new RangeFinder(new RangeFinderSimulationIO());
        intake = new Intake(new IntakeIOSimulation(), canRange);

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

        arm = new Arm(new ArmIO() {});
        elevator = new Elevator(new ElevatorIO() {}, arm);

        canRange = new RangeFinder(new RangeFinderIO() {});
        intake = new Intake(new IntakeIO() {}, canRange);
        break;
    }

    pathfinderVortechs =
        new PathfinderVortechs(
            Constants.CDrivetrain.DEFAULT_PATH_CONSTRAINTS, () -> drive.getPose());

    registerNamedCommandsAuto();

    autoChooser = AutoBuilder.buildAutoChooser();

    // set up the logged version of the auto chooser
    loggedAutoChooser = new LoggedDashboardChooser<>("Auto Choices", autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData(
        "CanRange Distance 0", (Sendable) this.arm.setCanrangeDistanceCommand(0));
    SmartDashboard.putData(
        "CanRange Distance 1000", (Sendable) this.arm.setCanrangeDistanceCommand(1000));

    SmartDashboard.putData("Elevator height 0", (Sendable) this.elevator.setTargetHeightCommand(0));
    SmartDashboard.putData("Elevator height 5", (Sendable) this.elevator.setTargetHeightCommand(5));

    SmartDashboard.putData("arm set target angle 0", (Sendable) this.arm.setTargetHeightCommand(0));
    SmartDashboard.putData("arm set target angle 5", (Sendable) this.arm.setTargetHeightCommand(5));

    SmartDashboard.putData(
        "arm set roller speed 1", (Sendable) this.arm.setRollerSpeedCommand(1, true));
    SmartDashboard.putData(
        "arm set roller speed -1", (Sendable) this.arm.setRollerSpeedCommand(-1, true));

    SmartDashboard.putData(
        "canrange set distance 100", (Sendable) this.canRange.setCanrangeDistanceCommand(100));
    SmartDashboard.putData(
        "canrange set distance 0", (Sendable) this.canRange.setCanrangeDistanceCommand(0));

    SmartDashboard.putData(
        "intake set power 100", (Sendable) this.intake.setTargetSpeedCommand(100));
    SmartDashboard.putData("intake set power 0", (Sendable) this.intake.setTargetSpeedCommand(0));

    SmartDashboard.putData(
        "intake until canrange",
        (Sendable) this.intake.intakeUntilCanRangeIsDetected(100, 10, true));
    ;

    arm.setDefaultCommand(arm.setRollerSpeedCommand(0, true));

    List<Pose2d> pathPoses = new ArrayList<>();
    pathPoses.add(new Pose2d());
    pathPoses.add(new Pose2d(10, 2, new Rotation2d()));
    VortechsClosestPoseSupplier poseSupplier =
        new VortechsClosestPoseSupplier(pathPoses, () -> drive.getPose());

    Command command =
        pathfinderVortechs
            .runPathCommand(() -> poseSupplier.getClosestPose())
            .alongWith(
                new WaitUntilCommand(() -> VortechsUtil.hasReachedDistance(0.2, pathfinderVortechs))
                    .andThen(arm.setTargetHeightCommandConsistentEnd(5))
                    .andThen(elevator.setTargetHeightCommand(5)));

    SmartDashboard.putData("run auto routine", (Sendable) command);
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
  } // end configure bindings

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return loggedAutoChooser.get();
  }

  // registers pathplanner's named commands
  private void registerNamedCommandsAuto() {}

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

  public Arm getArm() {
    return arm;
  }

  public Elevator getElevator() {
    return elevator;
  }
}
