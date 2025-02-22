package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ManualElevatorCommand extends Command {
  private final Elevator elevator;
  private final DoubleSupplier speedSupplier;

  /**
   * @param elevator The elevator subsystem.
   * @param speedSupplier A supplier for the desired speed (e.g. joystick axis).
   */
  public ManualElevatorCommand(Elevator elevator, DoubleSupplier speedSupplier) {
    this.elevator = elevator;
    this.speedSupplier = speedSupplier;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    // Directly command the elevator with the joystick value.
    elevator.setManualSpeed(speedSupplier.getAsDouble());
    System.out.println("Manual mode is in session");
  }

  @Override
  public void end(boolean interrupted) {
    // When manual control ends, hold the current position.
    elevator.holdPositionBrake();
  }
}
