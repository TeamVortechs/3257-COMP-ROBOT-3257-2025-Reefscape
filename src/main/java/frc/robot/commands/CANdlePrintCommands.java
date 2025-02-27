package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.CANdleSystem;

public class CANdlePrintCommands {
  public static class PrintVBat extends InstantCommand {
    public PrintVBat(CANdleSystem candleSystem) {
      super(() -> System.out.println("Vbat is " + candleSystem.getVbat() + "V"), candleSystem);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  public static class Print5V extends InstantCommand {
    public Print5V(CANdleSystem candleSystem) {
      super(() -> System.out.println("5V is " + candleSystem.get5V() + "V"), candleSystem);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  public static class PrintCurrent extends InstantCommand {
    public PrintCurrent(CANdleSystem candleSystem) {
      super(
          () -> System.out.println("Current is " + candleSystem.getCurrent() + "A"), candleSystem);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }

  public static class PrintTemperature extends InstantCommand {
    public PrintTemperature(CANdleSystem candleSystem) {
      super(
          () -> System.out.println("Temperature is " + candleSystem.getTemperature() + "C"),
          candleSystem);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
  }
}
