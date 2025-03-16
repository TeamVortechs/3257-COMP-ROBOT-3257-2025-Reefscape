package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.KDoublePreferences.PElevator;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Implementation of ElevatorModuleIO using two TalonFX motor controllers. The two motors drive the
 * same gear but are mounted oppositely.
 */
public class ElevatorModuleTalonFXIO implements ElevatorModuleIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  @AutoLogOutput private double leftHeight;
  // @AutoLogOutput private double rightHeight;

  /** Constructs the TalonFX-based elevator module. */
  public ElevatorModuleTalonFXIO(int motorIDLeft, int motorIDRight, String canbusName) {
    this.leftMotor = new TalonFX(motorIDLeft, canbusName);
    this.rightMotor = new TalonFX(motorIDRight, canbusName);

    TalonFXConfiguration elevatorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(80))
                    .withStatorCurrentLimitEnable(true));

    var slot0Configs = elevatorConfigs.Slot0;
    slot0Configs.kG = PElevator.kG.getValue();
    slot0Configs.kS = PElevator.kS.getValue();
    slot0Configs.kV = PElevator.kV.getValue();
    slot0Configs.kA = PElevator.kA.getValue();
    slot0Configs.kP = PElevator.kP.getValue();
    slot0Configs.kI = PElevator.kI.getValue();
    slot0Configs.kD = PElevator.kD.getValue();

    // var motionMagicConfigs = elevatorConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity =
    //     PElevator.speedLimit.getValue(); // Target cruise velocity of 80 rps
    // motionMagicConfigs.MotionMagicAcceleration =
    //     PElevator.accelerationLimit.getValue(); // Target acceleration of 160 rps/s (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk =
    //     PElevator.jerkLimit.getValue(); // Target jerk of 1600 rps/s/s (0.1 seconds)
    // motionMagicConfigs.MotionMagicExpo_kV = 0.12;
    // motionMagicConfigs.MotionMagicExpo_kA = 0.1;

    leftMotor.getConfigurator().apply(elevatorConfigs);
    rightMotor.getConfigurator().apply(elevatorConfigs);

    // Set both motors to Brake mode by default.
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    rightMotor.setControl(new Follower(motorIDLeft, false));
  }

  // advantage kit logging stuff(everything in here gets logged every tick)
  @Override
  public void updateInputs(ElevatorModuleIOInputsAutoLogged inputs) {
    boolean updatePID =
        false; // set to true to continuously update the PID constants through preferences
    if (updatePID) {
      TalonFXConfiguration elevatorConfigs =
          new TalonFXConfiguration()
              .withCurrentLimits(
                  new CurrentLimitsConfigs()
                      // Swerve azimuth does not require much torque output, so we can set a
                      // relatively
                      // low
                      // stator current limit to help avoid brownouts without impacting performance.
                      .withStatorCurrentLimit(Amps.of(80))
                      .withStatorCurrentLimitEnable(true));

      var slot0Configs = elevatorConfigs.Slot0;
      slot0Configs.kG = PElevator.kG.getValue();
      slot0Configs.kS = PElevator.kS.getValue();
      slot0Configs.kV = PElevator.kV.getValue();
      slot0Configs.kA = PElevator.kA.getValue();
      slot0Configs.kP = PElevator.kP.getValue();
      slot0Configs.kI = PElevator.kI.getValue();
      slot0Configs.kD = PElevator.kD.getValue();

      var motionMagicConfigs = elevatorConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity =
          PElevator.speedLimit.getValue(); // Target cruise velocity of 80 rps
      motionMagicConfigs.MotionMagicAcceleration =
          PElevator.accelerationLimit.getValue(); // Target acceleration of 160 rps/s (0.5 seconds)
      motionMagicConfigs.MotionMagicJerk =
          PElevator.jerkLimit.getValue(); // Target jerk of 1600 rps/s/s (0.1 seconds)

      leftMotor.getConfigurator().apply(elevatorConfigs);
      rightMotor.getConfigurator().apply(elevatorConfigs);

      // Set both motors to Brake mode by default.
      leftMotor.setNeutralMode(NeutralModeValue.Brake);
      rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    inputs.elevatorMotor1CurrentHeightMeter = leftMotor.get();
    inputs.elevatorMotor1CurrentHeightMeter = getHeightMeters(0);

    inputs.elevatorMotor1CurrentAmps = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.elevatorMotor1AppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();

    // inputs.elevatorMotor2CurrentHeightMeter = rightMotor.get();
    // inputs.elevatorMotor2CurrentHeightMeter = getHeightMeters(0);

    // inputs.elevatorMotor2CurrentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
    // inputs.elevatorMotor2AppliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();

    inputs.elevatorMotor1CurrentSpeedMeter = leftMotor.getVelocity().getValueAsDouble();
    // inputs.elevatorMotor2CurrentSpeedMeter = rightMotor.getVelocity().getValueAsDouble();

    inputs.isStalled = checkIfStalled();
  }

  /**
   * Returns the current elevator height in meters by averaging both motor encoders. this won't work
   * until we do the math with the gears to find out how much one rotation is in length
   */
  @Override
  public double getHeightMeters() {
    leftHeight = leftMotor.getPosition().getValueAsDouble();
    // rightHeight = rightMotor.getPosition().getValueAsDouble();

    // return (leftHeight + rightHeight) / 2.0;
    return leftHeight;
  }

  /** Sets the voltage to both motors. */
  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    // rightMotor.setVoltage(volts);

    // System.out.println("setting voltage to " + volts);
  }

  @Override
  public void PIDVoltage(double targetAngle) {
    final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

    // set target position to 100 rotations
    leftMotor.setControl(m_request.withPosition(targetAngle));
    // rightMotor.setControl(m_request.withPosition(targetAngle));
  }

  /** Resets both motor encoders to zero. */
  @Override
  public void resetEncoders() {
    leftMotor.setPosition(0);
    // rightMotor.setPosition(0);
  }

  /** Stops the motor immediately. */
  @Override
  public void stop() {
    leftMotor.stopMotor();
    // rightMotor.stopMotor();
  }

  /** returns true if either motor has exceeded 40 amps of torque current currently nonfunctional */
  @Override
  public boolean checkIfStalled() {

    return false;
  }

  /** Sets the speed of the motors (manual control mode). */
  @Override
  public void setSpeed(double speed) {
    // System.out.println("ModuleIO receiving this speed: " + speed);
    leftMotor.set(speed);
    // rightMotor.set(speed);
  }

  /** Sets the neutral mode for both motors (Brake or Coast). */
  @Override
  public void setBraked(boolean braked) {
    NeutralModeValue mode = braked ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    leftMotor.setNeutralMode(mode);
    rightMotor.setNeutralMode(mode); // fine since this is a config
  }

  @Override
  public double getVoltage() {
    return leftMotor.getMotorVoltage().getValueAsDouble();
  }
}
