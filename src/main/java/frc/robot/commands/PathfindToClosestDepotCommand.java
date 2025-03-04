package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/*
Names
brief description
 */
// THIS CLASS WILL BREAK THE ROBOT
public class PathfindToClosestDepotCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private Drive drive;

  private int targetPoseID = 0;

  private boolean lockedIn = false;

  private Command[] depotPathCommands;

  public PathfindToClosestDepotCommand(Drive drive) {
    // addRequirements(null);
    this.drive = drive;
    depotPathCommands = new Command[6];
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < depotPathCommands.length; i++) {
      depotPathCommands[i] = PathfindingCommands.pathfindToDepotCommand(i);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int curPoseID = PathfindingCommands.getClosestDepotPath(drive.getPose());

    if (!lockedIn) {
      targetPoseID = curPoseID;
      depotPathCommands[targetPoseID].schedule();
    }

    if (targetPoseID != curPoseID) {
      // depotPathCommands[targetPoseID].cancel();
      targetPoseID = curPoseID;
      depotPathCommands[targetPoseID].schedule();
      ;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    depotPathCommands[targetPoseID].cancel();

    // for(int i = 0; i < depotPathCommands.length; i++) {
    //   depotPathCommands[i].cancel();
    // }

    lockedIn = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return depotPathCommands[targetPoseID].isFinished();
  }
}
