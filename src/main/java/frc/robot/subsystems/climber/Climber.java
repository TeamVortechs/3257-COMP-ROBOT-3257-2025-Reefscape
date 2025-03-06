package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  // private DigitalInput limitswitch;

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
    // this.limitswitch = limitswitch;
  }
  
  @Override
  public void periodic() {
    climberIO.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public void setMotorSpeed(double speed) {
    if (Math.abs(speed) > KDoublePreferences.PClimber.manualSpeedLimit.getValue()) {
      speed = Math.copySign(KDoublePreferences.PClimber.manualSpeedLimit.getValue(), speed);
    }
    climberIO.setMotorSpeed(speed);
  }

  // public boolean isDetected(DigitalInput limitswitch) {
  //   boolean isdetected = limitswitch.get();
  //   return isdetected;
  // }
}
