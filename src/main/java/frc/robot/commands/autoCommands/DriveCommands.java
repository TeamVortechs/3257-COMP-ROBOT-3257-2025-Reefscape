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

package frc.robot.commands.autoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.2;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Robot-Centric drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command RobotCentricDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Do not convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          // boolean isFlipped =
          //     DriverStation.getAlliance().isPresent()
          //         && DriverStation.getAlliance().get() == Alliance.Red;
          // drive.runVelocity(
          //     ChassisSpeeds.fromFieldRelativeSpeeds(
          //         speeds,
          //         isFlipped
          //             ? drive.getRotation().plus(new Rotation2d(Math.PI))
          //             : drive.getRotation()));
          drive.runVelocity(speeds);
        },
        drive);
  }

  /**
   * robot-relative drive command that takes in a parameter of how many radians it should be turning
   * intended for usage with limelight's tx variable to feed into the rotation parameter for object
   * detection i'm aware that this is a VERY verbose name stems from joystickDriveAtAngle
   */
  public static Command RobotCentricDriveWhileTurningToAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {

              // variables to ensure that the PID behaves itself and doesn't try
              // to switch the wheels back and forth rapidly at low speed input
              double ACCEPTABLE_DEVIATION = 0.05; // acceptable angle deviation in radians
              double MINIMUM_LINEAR_VELOCITY =
                  0.1; // minimum magnitude of velocity to exceed (the name's a bit inaccurate)
              // Get linear velocity
              // if not greater than MINIMUM_LINEAR_VELOCITY then create a new translation2d
              // (effectively zeroes it out)
              // get linear magnitude and square it to mimic getLinearVelocityFromJoysticks
              double linearMagnitude =
                  MathUtil.applyDeadband(
                      Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
              linearMagnitude = linearMagnitude * linearMagnitude;
              Translation2d linearVelocity =
                  linearMagnitude > MINIMUM_LINEAR_VELOCITY
                      ? getLinearVelocityFromJoysticks(
                          xSupplier.getAsDouble(), ySupplier.getAsDouble())
                      : new Translation2d();

              // Calculate angular speed
              // since this is robot centric, the target should be radians deviating from the
              // current position
              // if rotation supplier is greater than ACCEPTABLE_DEVIATION, use PID; else, use zero
              double omega =
                  (Math.abs(rotationSupplier.get().getRadians()) > ACCEPTABLE_DEVIATION)
                      ? angleController.calculate(
                          drive.getRotation().getRadians(),
                          drive.getRotation().getRadians() + rotationSupplier.get().getRadians())
                      : 0;

              // Do not convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              // boolean isFlipped =
              //     DriverStation.getAlliance().isPresent()
              //         && DriverStation.getAlliance().get() == Alliance.Red;
              // drive.runVelocity(
              //     ChassisSpeeds.fromFieldRelativeSpeeds(
              //         speeds,
              //         isFlipped
              //             ? drive.getRotation().plus(new Rotation2d(Math.PI))
              //             : drive.getRotation()));
              drive.runVelocity(speeds);
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  // fully automatic drive system to center self with target piece and drive into it
  public static Command LimelightDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {

              // variables to ensure that the PID behaves itself and doesn't try
              // to switch the wheels back and forth rapidly at low speed input
              double ACCEPTABLE_DEVIATION = 0.08; // acceptable angle deviation in radians
              double MINIMUM_Y_VELOCITY = 0.1; // minimum *magnitude* of velocity
              double ACTIVATION_ANGLE =
                  ACCEPTABLE_DEVIATION * 4; // angle in rads when to start going forwards
              // Get linear velocity
              // if not greater than MINIMUM_LINEAR_VELOCITY then create a new translation2d
              // get linear magnitude and square it to mimic getLinearVelocityFromJoysticks
              //   double linearMagnitude =
              //       MathUtil.applyDeadband(
              //           Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
              //   linearMagnitude = linearMagnitude * linearMagnitude;
              //   Translation2d linearVelocity =
              //       linearMagnitude > MINIMUM_LINEAR_VELOCITY
              //           ? getLinearVelocityFromJoysticks(
              //               xSupplier.getAsDouble(), ySupplier.getAsDouble())
              //           : new Translation2d();
              // linear velocity to use when ready to activate forward motion
              // if y supplier is below minimum, then use 0 instead
              Translation2d preparedLinVel =
                  ySupplier.getAsDouble() > MINIMUM_Y_VELOCITY
                      ? getLinearVelocityFromJoysticks(
                          xSupplier.getAsDouble(), ySupplier.getAsDouble())
                      : getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), 0);

              // Calculate angular speed
              // if rotation supplier is greater than ACCEPTABLE_DEVIATION, use PID; else, use zero
              double omega =
                  (Math.abs(rotationSupplier.get().getRadians()) > ACCEPTABLE_DEVIATION)
                      ? angleController.calculate(
                          drive.getRotation().getRadians(),
                          drive.getRotation().getRadians() + rotationSupplier.get().getRadians())
                      : 0;

              // set chassis speeds and run them
              // if within acceptable turn and lateral movement, use preparedlinvel (forward
              // movement)
              // else, use only y component and set x (forward) to 0
              ChassisSpeeds speeds;
              if (Math.abs(omega) < ACTIVATION_ANGLE) {
                speeds =
                    new ChassisSpeeds(
                        preparedLinVel.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        preparedLinVel.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega);
              } else {
                speeds =
                    new ChassisSpeeds(
                        0, preparedLinVel.getY() * drive.getMaxLinearSpeedMetersPerSec(), omega);
              }
              // debug putters
              SmartDashboard.putNumber("prepped x: ", preparedLinVel.getX());
              SmartDashboard.putNumber("chosen y: ", preparedLinVel.getY());
              SmartDashboard.putNumber("chosen omega: ", omega);
              System.out.println(
                  "speed variables: "
                      + preparedLinVel.getX()
                      + " "
                      + preparedLinVel.getY()
                      + " "
                      + omega);
              System.out.println(
                  "input variables: " + xSupplier.getAsDouble() + " " + ySupplier.getAsDouble());
              drive.runVelocity(speeds);
            },
            drive)
        // interrupt this command if the limelight loses its target
        .onlyWhile(() -> LimelightHelpers.getTA("") > 0)
        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /*
   * command that chooses whether or not to use limelight drive
   * if limelight TA is greater than 0 (meaning a target is there) then use limelight drive
   * otherwise, return the normal joystick drive command
   */
  public static Command ChooseIfLimelightDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier limelightXSupplier,
      DoubleSupplier limelightYSupplier,
      Supplier<Rotation2d> limelightRotationSupplier) {

    if (LimelightHelpers.getTA("") > 0) {
      return LimelightDrive(
          drive, limelightXSupplier, limelightYSupplier, limelightRotationSupplier);
    } else {
      return joystickDrive(drive, xSupplier, ySupplier, omegaSupplier);
    }
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
