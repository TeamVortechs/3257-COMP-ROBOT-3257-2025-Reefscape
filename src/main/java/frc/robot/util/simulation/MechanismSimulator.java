package frc.robot.util.simulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class MechanismSimulator {
  private final LoggedMechanism2d armPanel;
  private final LoggedMechanismRoot2d armRoot;
  private final LoggedMechanismLigament2d armLigament;

  private final LoggedMechanism2d elevatorPanel;
  private final LoggedMechanismRoot2d elevatorRoot;
  private final LoggedMechanismLigament2d elevatorLigament;

  private final Arm arm;

  private final Elevator elevator;

  public MechanismSimulator(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;

    // initialize elevator
    this.elevatorPanel = new LoggedMechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(100));
    this.elevatorRoot =
        elevatorPanel.getRoot("elevator", Units.inchesToMeters(40), Units.inchesToMeters(0));
    this.elevatorLigament =
        elevatorRoot.append(
            new LoggedMechanismLigament2d(
                "arm", Units.inchesToMeters(25), 90, 6, new Color8Bit(Color.kYellow)));

    // initialize elevator
    this.armPanel = new LoggedMechanism2d(Units.inchesToMeters(60), Units.inchesToMeters(100));
    this.armRoot = armPanel.getRoot("arm", Units.inchesToMeters(7.35), Units.inchesToMeters(10));

    this.armLigament =
        armRoot.append(
            new LoggedMechanismLigament2d(
                "arm", Units.inchesToMeters(25), 90, 6, new Color8Bit(Color.kYellow)));
  }

  public void periodic() {
    double angle = arm.getCurrentAngle();
    double height = elevator.getCurrentHeight();

    // sets angles/heights
    armLigament.setAngle(90 - angle * 22);
    elevatorLigament.setLength(height / 60 + 0.5);

    // set the arm to move up and down with the elevator
    armRoot.setPosition(Units.inchesToMeters(40), elevatorLigament.getLength());

    // changes the arm color depending on wether the rollers go forwards or backwards
    // intaking/keeping in
    if (Math.abs(arm.getRollerSpeed()) < 1) {
      armLigament.setColor(new Color8Bit(Color.kBlue));
      // ejecting
    } else if (arm.getRollerSpeed() > 0) {
      armLigament.setColor(new Color8Bit(Color.kGreen));

      // doing nothing
    } else {
      armLigament.setColor(new Color8Bit(Color.kRed));
    }

    // records both mechanisms
    Logger.recordOutput("robot arm mechanism", armPanel);
    Logger.recordOutput("robot elevator mechanism", elevatorPanel);
  }
}
