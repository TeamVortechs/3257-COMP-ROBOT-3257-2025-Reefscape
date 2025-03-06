package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.KDoublePreferences.PWrist;
import org.littletonrobotics.junction.AutoLogOutput;

public class WristIOTalonFX implements WristIO {
  private TalonFX arm;
  private TalonFX rollers;
  private CANrange canRange;

  private boolean updateInputs = true;

  @AutoLogOutput private double angle;

  public WristIOTalonFX(int armID, int rollerID, String canbusName) {

    this.arm = new TalonFX(armID, canbusName);
    this.rollers = new TalonFX(rollerID, canbusName);

    // this.canRange = new CANrange(CanrangeID);
    TalonFXConfiguration armMotorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(Constants.Arm.WRIST_STATOR_LIMIT))
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    var slot0Configs = armMotorConfigs.Slot0;
    slot0Configs.kS = PWrist.kS.getValue(); // Add 0.25 V output to overcome static friction
    slot0Configs.kV = PWrist.kV.getValue(); // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = PWrist.kA.getValue(); // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP =
        PWrist.kP.getValue(); // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = PWrist.kI.getValue(); // no output for integrated error
    slot0Configs.kD = PWrist.kD.getValue(); // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = armMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = PWrist.speedLimit.getValue();
    motionMagicConfigs.MotionMagicAcceleration = PWrist.accelerationLimit.getValue();
    motionMagicConfigs.MotionMagicJerk = PWrist.jerkLimit.getValue();

    arm.getConfigurator().apply(armMotorConfigs);

    // Set motor to Brake mode by default.
    arm.setNeutralMode(NeutralModeValue.Brake);
    

    TalonFXConfiguration rollerMotorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(Constants.Arm.WRIST_STATOR_LIMIT))
                    .withStatorCurrentLimitEnable(true));
    rollers.getConfigurator().apply(rollerMotorConfigs);

    // Set motor to Brake mode by default.
    rollers.setNeutralMode(NeutralModeValue.Brake);
  }

  // sets the PID target angle
  @Override
  public void PIDVoltage(double targetAngle) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    arm.setControl(m_request.withPosition(targetAngle));
    // System.out.println("Voltage being sent in PID Voltage");
  }

  // sets the speed of the arm
  @Override
  public void setArmSpeed(double speed) {
    arm.set(speed);
  }

  // sets the speed of the orllers
  public void setRollerSpeed(double speed) {
    rollers.set(speed);
  }

  // gets the angle of the arm
  @Override
  public double getAngleRotations() {
    angle = arm.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("Wrist angle: ", angle);
    return angle;
  }

  // advantage kti logging stuff
  @Override
  public void updateInputs(WristIOInputsAutoLogged inputs) {

    if(updateInputs) {
        TalonFXConfiguration armMotorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(Constants.Arm.WRIST_STATOR_LIMIT))
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    var slot0Configs = armMotorConfigs.Slot0;
    slot0Configs.kS = PWrist.kS.getValue(); // Add 0.25 V output to overcome static friction
    slot0Configs.kV = PWrist.kV.getValue(); // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = PWrist.kA.getValue(); // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP =
        PWrist.kP.getValue(); // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = PWrist.kI.getValue(); // no output for integrated error
    slot0Configs.kD = PWrist.kD.getValue(); // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = armMotorConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = PWrist.speedLimit.getValue();
    motionMagicConfigs.MotionMagicAcceleration = PWrist.accelerationLimit.getValue();
    motionMagicConfigs.MotionMagicJerk = PWrist.jerkLimit.getValue();

    arm.getConfigurator().apply(armMotorConfigs);
    }



    inputs.wristAppliedVoltage = arm.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps = arm.getStatorCurrent().getValueAsDouble();
    inputs.wristSpeedRotations = arm.get();
    inputs.wristLocationRotations = getAngleRotations();

    // inputs.canRangeDistance = canRange.getDistance().getValueAsDouble();

    inputs.rollersCurrent = rollers.getStatorCurrent().getValueAsDouble();
    inputs.rollersEncoder = rollers.getPosition().getValueAsDouble();
    inputs.rollersSpeed = rollers.get();
  }

  // sets the arm motor to brake(DOES ACTUALY STOP THE ARM MOTORO)
  @Override
  public void setBraked(boolean braked) {
    if (braked) {
      arm.setNeutralMode(NeutralModeValue.Brake);
    } else {
      arm.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  // returns wether or not the can range is detected
  // @Override
  // public boolean isDetected() {
  //   return canRange.getIsDetected().getValue();
  // }

  // gets the distance from the can range
  // @Override
  // public double getDistance() {
  //   return canRange.getDistance().getValueAsDouble();
  // }

  // sets the voltage of the arm
  @Override
  public void setArmVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  // resets the arm encoder
  @Override
  public void zeroArmEncoder() {
    arm.setPosition(0);
  }
}
