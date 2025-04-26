package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Implementation of ElevatorModuleIO using two TalonFX motor controllers. The two motors drive the
 * same gear but are mounted oppositely.
 */
public class ElevatorModuleTalonFXIOSimulation extends ElevatorModuleTalonFXIO {
  private final TalonFXSimState leftMotorSim;
  private final TalonFXSimState rightMotorSim;
  private final DCMotorSim elevatorGearboxSim;
  private final double kGearRatio = 1;

  @AutoLogOutput private double leftHeight;
  @AutoLogOutput private double rightHeight;

  /** Constructs the TalonFX-based elevator module. */
  public ElevatorModuleTalonFXIOSimulation(int motorIDLeft, int motorIDRight, String canbusName) {
    super(motorIDLeft, motorIDLeft, canbusName);
    leftMotorSim = super.getLeftMotor().getSimState();
    rightMotorSim = super.getRightMotor().getSimState();
    elevatorGearboxSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2), 0.001, kGearRatio),
            DCMotor.getKrakenX60(2));

    leftMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    rightMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    // this.leftMotor = new TalonFX(motorIDLeft, canbusName);
    // this.rightMotor = new TalonFX(motorIDRight, canbusName);

    // TalonFXConfiguration elevatorConfigs =
    //     new TalonFXConfiguration()
    //         .withCurrentLimits(
    //             new CurrentLimitsConfigs()
    //                 // Swerve azimuth does not require much torque output, so we can set a
    //                 // relatively
    //                 // low
    //                 // stator current limit to help avoid brownouts without impacting
    // performance.
    //                 .withStatorCurrentLimit(Amps.of(80))
    //                 .withStatorCurrentLimitEnable(true));

    // var slot0Configs = elevatorConfigs.Slot0;
    // slot0Configs.kG = PElevator.kG.getValue();
    // slot0Configs.kS = PElevator.kS.getValue();
    // slot0Configs.kV = PElevator.kV.getValue();
    // slot0Configs.kA = PElevator.kA.getValue();
    // slot0Configs.kP = PElevator.kP.getValue();
    // slot0Configs.kI = PElevator.kI.getValue();
    // slot0Configs.kD = PElevator.kD.getValue();

    // var motionMagicConfigs = elevatorConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity =
    //     PElevator.speedLimit.getValue(); // Target cruise velocity of 80 rps
    // motionMagicConfigs.MotionMagicAcceleration =
    //     PElevator.accelerationLimit.getValue(); // Target acceleration of 160 rps/s (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk =
    //     PElevator.jerkLimit.getValue(); // Target jerk of 1600 rps/s/s (0.1 seconds)
    // motionMagicConfigs.MotionMagicExpo_kV = 0.12;
    // motionMagicConfigs.MotionMagicExpo_kA = 0.1;

    // leftMotor.getConfigurator().apply(elevatorConfigs);
    // rightMotor.getConfigurator().apply(elevatorConfigs);

    // // Set both motors to Brake mode by default.
    // leftMotor.setNeutralMode(NeutralModeValue.Brake);
    // rightMotor.setNeutralMode(NeutralModeValue.Brake);

    // rightMotor.setControl(new Follower(motorIDLeft, false));
  }

  // advantage kit logging stuff(everything in here gets logged every tick)
  @Override
  public void updateInputs(ElevatorModuleIOInputsAutoLogged inputs) {
    super.updateInputs(inputs);

    // update the simulation
    leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    final double[] voltageInputs = {
      leftMotorSim.getMotorVoltage(), rightMotorSim.getMotorVoltage()
    };
    elevatorGearboxSim.setInput(voltageInputs);

    elevatorGearboxSim.update(0.02);

    final double leftPos = elevatorGearboxSim.getOutput().get(0, 0);
    final double rightPos = elevatorGearboxSim.getOutput().get(1, 0);
    final double leftVel = elevatorGearboxSim.getAngularVelocity().in(RotationsPerSecond);
    final double rightVel = elevatorGearboxSim.getAngularVelocity().in(RotationsPerSecond);

    leftMotorSim.setRawRotorPosition(leftPos * kGearRatio);
    leftMotorSim.setRotorVelocity(leftVel * kGearRatio);
    rightMotorSim.setRawRotorPosition(rightPos * kGearRatio);
    rightMotorSim.setRotorVelocity(rightVel * kGearRatio);
    // boolean updatePID =
    //     false; // set to true to continuously update the PID constants through preferences
    // if (updatePID) {
    //   TalonFXConfiguration elevatorConfigs =
    //       new TalonFXConfiguration()
    //           .withCurrentLimits(
    //               new CurrentLimitsConfigs()
    //                   // Swerve azimuth does not require much torque output, so we can set a
    //                   // relatively
    //                   // low
    //                   // stator current limit to help avoid brownouts without impacting
    // performance.
    //                   .withStatorCurrentLimit(Amps.of(80))
    //                   .withStatorCurrentLimitEnable(true));

    //   var slot0Configs = elevatorConfigs.Slot0;
    //   slot0Configs.kG = PElevator.kG.getValue();
    //   slot0Configs.kS = PElevator.kS.getValue();
    //   slot0Configs.kV = PElevator.kV.getValue();
    //   slot0Configs.kA = PElevator.kA.getValue();
    //   slot0Configs.kP = PElevator.kP.getValue();
    //   slot0Configs.kI = PElevator.kI.getValue();
    //   slot0Configs.kD = PElevator.kD.getValue();

    //   var motionMagicConfigs = elevatorConfigs.MotionMagic;
    //   motionMagicConfigs.MotionMagicCruiseVelocity =
    //       PElevator.speedLimit.getValue(); // Target cruise velocity of 80 rps
    //   motionMagicConfigs.MotionMagicAcceleration =
    //       PElevator.accelerationLimit.getValue(); // Target acceleration of 160 rps/s (0.5
    // seconds)
    //   motionMagicConfigs.MotionMagicJerk =
    //       PElevator.jerkLimit.getValue(); // Target jerk of 1600 rps/s/s (0.1 seconds)

    //   leftMotorSim.getConfigurator().apply(elevatorConfigs);
    //   rightMotorSim.getConfigurator().apply(elevatorConfigs);

    //   // Set both motors to Brake mode by default.
    //   leftMotorSim.setNeutralMode(NeutralModeValue.Brake);
    //   rightMotorSim.setNeutralMode(NeutralModeValue.Brake);
    // }
    // inputs.elevatorMotor1CurrentHeightMeter = leftMotorSim.get();
    // inputs.elevatorMotor1CurrentHeightMeter = getHeightMeters(0);

    // inputs.elevatorMotor1CurrentAmps = leftMotorSim.getStatorCurrent().getValueAsDouble();
    // inputs.elevatorMotor1AppliedVolts = leftMotorSim.getMotorVoltage().getValueAsDouble();

    // inputs.elevatorMotor2CurrentHeightMeter = rightMotorSim.get();
    // inputs.elevatorMotor2CurrentHeightMeter = getHeightMeters(0);

    // inputs.elevatorMotor2CurrentAmps = rightMotorSim.getStatorCurrent().getValueAsDouble();
    // inputs.elevatorMotor2AppliedVolts = rightMotorSim.getMotorVoltage().getValueAsDouble();

    // inputs.elevatorMotor1CurrentSpeedMeter = leftMotorSim.getVelocity().getValueAsDouble();
    // inputs.elevatorMotor2CurrentSpeedMeter = rightMotorSim.getVelocity().getValueAsDouble();

    // inputs.isStalled = checkIfStalled();

  }

  // /**
  //  * Returns the current elevator height in meters by averaging both motor encoders. this won't
  // work
  //  * until we do the math with the gears to find out how much one rotation is in length
  //  */
  // @Override
  // public double getHeightMeters() {
  //   leftHeight = leftMotorSim.getPosition().getValueAsDouble();
  //   rightHeight = rightMotorSim.getPosition().getValueAsDouble();

  //   return (leftHeight + rightHeight) / 2.0;
  // }

  // /** Sets the voltage to both motors. */
  // @Override
  // public void setVoltage(double volts) {
  //   leftMotorSim.setVoltage(volts);
  //   // rightMotor.setVoltage(volts);

  //   // System.out.println("setting voltage to " + volts);
  // }

  // @Override
  // public void PIDVoltage(double targetAngle) {
  //   final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

  //   // set target position to 100 rotations
  //   leftMotorSim.setControl(m_request.withPosition(targetAngle));
  //   // rightMotor.setControl(m_request.withPosition(targetAngle));
  // }

  // /** Resets both motor encoders to zero. */
  // @Override
  // public void resetEncoders() {
  //   leftMotorSim.setPosition(0);
  //   rightMotorSim.setPosition(0);
  //   // rightMotor.setControl(new Follower(Constants.Elevator.MOTOR_LEFT_ID, false));
  // }

  // /** Stops the motor immediately. */
  // @Override
  // public void stop() {
  //   leftMotorSim.stopMotor();
  //   // rightMotor.stopMotor();
  // }

  // /** returns true if either motor has exceeded 40 amps of torque current currently nonfunctional
  // */
  // @Override
  // public boolean checkIfStalled() {

  //   return false;
  // }

  // /** Sets the speed of the motors (manual control mode). */
  // @Override
  // public void setSpeed(double speed) {
  //   // System.out.println("ModuleIO receiving this speed: " + speed);
  //   leftMotorSim.set(speed);
  //   // rightMotor.set(speed);
  // }

  // /** Sets the neutral mode for both motors (Brake or Coast). */
  // @Override
  // public void setBraked(boolean braked) {
  //   NeutralModeValue mode = braked ? NeutralModeValue.Brake : NeutralModeValue.Coast;
  //   leftMotorSim.setNeutralMode(mode);
  //   rightMotorSim.setNeutralMode(mode);
  // }

  // @Override
  // public double getVoltage() {
  //   return leftMotorSim.getMotorVoltage().getValueAsDouble();
  // }

  // @Override
  // public double getCurrent() {
  //   return leftMotorSim.getStatorCurrent().getValueAsDouble();
  // }
}
