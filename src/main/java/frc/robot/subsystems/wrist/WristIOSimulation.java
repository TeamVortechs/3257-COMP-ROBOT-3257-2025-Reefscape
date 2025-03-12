package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.KDoublePreferences.PWrist;
import org.littletonrobotics.junction.AutoLogOutput;

public class WristIOSimulation implements WristIO {
  private final DCMotorSim armMotorsSim;
  private final DCMotorSim rollerMotorsSim;

  private final double speedDivider = 1;

  private PIDController armPIDController =
      new PIDController(
          PWrist.kP.getValue() / speedDivider,
          PWrist.kI.getValue() / speedDivider,
          PWrist.kD.getValue() / speedDivider);
  private ArmFeedforward armFeedforward =
      new ArmFeedforward(
          PWrist.kS.getValue() / speedDivider,
          PWrist.kG.getValue() / speedDivider,
          PWrist.kV.getValue() / speedDivider);

  private double targetVel = PWrist.speedLimit.getValue() / speedDivider;

  // private double targetVel = PWrist.speedLimit.getValue();
  private double targetAccel = PWrist.accelerationLimit.getValue() / speedDivider;

  @AutoLogOutput private double angle;

  public WristIOSimulation() {
    this.armMotorsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
            DCMotor.getKrakenX60(1));
    this.rollerMotorsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
            DCMotor.getKrakenX60(1));
  }

  // sets the PID target angle
  @Override
  public void PIDVoltage(double targetAngle) {
    double currentAngle = armMotorsSim.getAngularPositionRotations();
    double inputVoltage =
        armPIDController.calculate(currentAngle, targetAngle)
            + armFeedforward.calculate(targetAngle * Math.PI * 2 - (Math.PI / 2), targetVel);
    // System.out.println("Input volt: "+inputVoltage+" Target Angle: "+targetAngle);
    setArmVoltage(inputVoltage);
    // System.out.println("Voltage being sent in PID Voltage");
  }

  // sets the speed of the arm
  // @Override
  // public void setArmSpeed(double speed) {
  //   arm.set(speed);
  // }

  // sets the speed of the orllers
  public void setRollerSpeed(double speed) {
    rollerMotorsSim.setInputVoltage(speed * 12); // lol i hope that works
  }

  // gets the angle of the arm
  @Override
  public double getAngleRotations() {
    angle = armMotorsSim.getAngularPositionRotations();
    SmartDashboard.putNumber("Wrist angle: ", angle);
    return angle;
  }

  // advantage kti logging stuff
  @Override
  public void updateInputs(WristIOInputsAutoLogged inputs) {
    inputs.wristAppliedVoltage = armMotorsSim.getInputVoltage();
    inputs.wristCurrentAmps = armMotorsSim.getCurrentDrawAmps();
    inputs.wristSpeedRotations = armMotorsSim.getAngularVelocityRPM();
    inputs.wristLocationRotations = getAngleRotations();

    // inputs.canRangeDistance = canRange.getDistance().getValueAsDouble();
    // wonder why there's no rollers voltage here.
    inputs.rollersCurrent = rollerMotorsSim.getCurrentDrawAmps();
    inputs.rollersEncoder = rollerMotorsSim.getAngularPositionRotations();
    inputs.rollersSpeed = rollerMotorsSim.getAngularVelocityRPM();

    armMotorsSim.update(0.02);
    rollerMotorsSim.update(0.02);
  }

  // sets the arm motor to brake(DOES ACTUALY STOP THE ARM MOTORO)
  // @Override
  // public void setBraked(boolean braked) {
  //   if (braked) {
  //     arm.setNeutralMode(NeutralModeValue.Brake);
  //   } else {
  //     arm.setNeutralMode(NeutralModeValue.Coast);
  //   }
  // }

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
    // System.out.println("Set Arm Voltage called.");
    armMotorsSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  // resets the arm encoder
  @Override
  public void zeroArmEncoder() {
    armMotorsSim.setAngle(0);
  }

  @Override
  public double getRollerSpeed() {
    return rollerMotorsSim.getAngularVelocityRPM();
  }
}
