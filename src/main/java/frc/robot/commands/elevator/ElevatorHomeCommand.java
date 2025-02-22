package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorHomeCommand extends Command {

  private final Elevator elevator;

  public ElevatorHomeCommand(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator); // Ensure this command owns the Elevator subsystem while running.
  }

  @Override
  public void initialize() {
    // Optionally log that homing is starting.
    System.out.println("Starting elevator homing procedure.");
  }

  @Override
  public void execute() {
    // Drive downward slowly. (Negative speed for down.)
    elevator.setManualSpeed(-0.2);
  }

  @Override
  public boolean isFinished() {
    // Finish when the current height is near zero.
    return elevator.getCurrentHeight() < 0.02;
  }

  @Override
  public void end(boolean interrupted) {
    // Hold position once homing is complete.
    elevator.holdPositionBrake();
    System.out.println("Elevator homed. Encoder reset.");
  }
}
