package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSimulation implements IntakeIO {
    private final DCMotorSim motorSim;

    private PIDController controller;

    protected double tolerance = 0.1;
    protected double targetPosition = 0;
    protected double maxPosition = 0.0;

    public IntakeIOSimulation() {

      this.motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1),
          DCMotor.getKrakenX60(1));

      rebuildMotorsPID();
    }

    // updates the given inputs with new values(advantage kit stuff)
    public void updateInputs(IntakeIOInputsAutoLogged inputsAutoLogged) {
      updatePID();
    }

    // getters for motors

    // gets the height of the arm in meters
    public double getCurrent() {
        return motorSim.getCurrentDrawAmps();
    }

    public double getVoltage() {
        return motorSim.getInputVoltage();
    }

    // setters for motors
    public void setVoltage(double volt) {
      motorSim.setInputVoltage(MathUtil.clamp(volt, -12, 12));
    }

    // sets the position of the rollers. This function will most likely not be implemented
    public void setRotationTarget(double position) {
      targetPosition = position;
    }

    // misc methods

    // rebuilds the pid constants of the motors
    public void rebuildMotorsPID() {
      controller = new PIDController(0.9, 0, 0.1);
    }

    /** Stops the motor immediately */
    public void stop() {
      setVoltage(0);
    };

    public void resetEncoders() {
      motorSim.setAngle(0);
    }

    public void setBraked(boolean braked) {

    }

    // gets the highest possible height of the arm in radians
    public  double getMaxRotations() {
        return maxPosition;
    }

    // gets the height of the arm in meters
    public  double getRotations() {
        return motorSim.getAngularPositionRotations();
    }

    public boolean isMaxPosition() {
      return getMaxRotations() - getRotations() < tolerance;
    }

    public void setSpeedTarget(double speed) {
      motorSim.setAngularVelocity(speed * Math.PI * 2);
    }

    public double getTargetSpeed() {
        return motorSim.getAngularVelocityRadPerSec() * Math.PI * 2;
    }

    protected void updatePID() {
      double currentAngle = motorSim.getAngularPositionRotations();
      double inputVoltage =
          controller.calculate(currentAngle, targetPosition);
      // System.out.println("Input volt: "+inputVoltage+" Target Angle: "+targetAngle);
      setVoltage(inputVoltage);
    }
}