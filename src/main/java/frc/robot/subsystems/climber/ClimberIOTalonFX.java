package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX climberMotor;

  public ClimberIOTalonFX(int climberMotorID){
    climberMotor = new TalonFX(climberMotorID);
  }

  public void SetMotorSpeed(double Speed) {
    climberMotor.set(Speed);
  }

  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    inputs.CurrentMotorSpeed = climberMotor.get();
    inputs.CurrentMotorVoltage = climberMotor.getMotorVoltage(true).getValueAsDouble();
    inputs.CurrentMotorAmps = climberMotor.getStatorCurrent().getValueAsDouble();
  }
}
