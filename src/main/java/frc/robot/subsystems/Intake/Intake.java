package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged inputsAutoLogged = new IntakeIOInputsAutoLogged();
  private DigitalInput limitswitch;

  public Intake(IntakeIO intakeIO, DigitalInput limitswitch) {
    this.intakeIO = intakeIO;
    this.limitswitch = limitswitch;
  }

  public void SetMotorSpeed(double speed) {
    intakeIO.setMotorSpeed(speed);
  }

  public boolean isDetected(DigitalInput limitswitch) {
    boolean isdetected = limitswitch.get();
    return isdetected;
  }
}
