package frc.robot.commands.pathfindingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * PathfindToClosestDepotCommand is a dynamic command that continuously monitors the robot's pose
 * and selects an appropriate pathfinding command to navigate to the closest depot.
 *
 * <p>Improvements Made: - Removed pre-created command array and replaced it with dynamic command
 * creation. - Removed the 'lockedIn' flag; instead, the target is managed via targetPoseID. -
 * Properly cancels the active command when a new target is selected or when the command ends.
 */
public class PathfindToClosestDepotCommand extends Command {

  private final Drive drive;
  private Command activeCommand; // Dynamically created command instance
  // private int targetPoseID = -1; // -1 indicates no target selected yet

  private boolean scheduledPath = false;

  private PathfindingCommandsHelp pathfindingCommandsHelp;

  /**
   * Constructs a new PathfindToClosestDepotCommand.
   *
   * @param drive the drive subsystem.
   * @param left whether to use left-side depot paths.
   */
  public PathfindToClosestDepotCommand(Drive drive, String... paths) {
    this.drive = drive;
    pathfindingCommandsHelp = new PathfindingCommandsHelp(paths);
    // Declare dependency on the drive subsystem.
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset state on initialization.
    // targetPoseID = -1;
    activeCommand = null;
    scheduledPath = false;
  }

  @Override
  public void execute() {
    // Determine the closest depot path index based on the current robot pose.
    // int curPoseID = PathfindingCommands.getClosestDepotPath(drive.getPose(), left);

    // If the depot target has changed, cancel the previous command and schedule a new one.
    // if (curPoseID != targetPoseID) {
    //   if (activeCommand != null && activeCommand.isScheduled()) {
    //     activeCommand.cancel();
    //   }
    //   targetPoseID = curPoseID;
    //   // Create a new command instance using the factory method.
    //   activeCommand = PathfindingCommands.pathfindToDepotCommand(targetPoseID, left);
    //   activeCommand.schedule();
    // }

    if (scheduledPath == false) {
      int curPoseID = pathfindingCommandsHelp.getClosestDepotPath(drive.getPose());

      if (activeCommand != null && activeCommand.isScheduled()) {
        activeCommand.cancel();
      }

      activeCommand = pathfindingCommandsHelp.pathfindToDepotCommand(curPoseID);
      activeCommand.schedule();

      scheduledPath = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Cancel the active command if it is still scheduled.
    // System.out.println(
    // "in the end function of the pathfind to the closest depot command\n\n\n\n\n\n");

    if (activeCommand != null && activeCommand.isScheduled()) {
      activeCommand.cancel();

      // System.out.println("ending the pathfind to closest depot command\n\n\n\n\n\n\n\n");
    }
  }

  @Override
  public boolean isFinished() {
    // Consider this command finished when the active command has completed.
    return activeCommand != null && activeCommand.isFinished();
  }
}
