package frc.robot.util.FieldMovement.vortechsPath;

import java.util.TreeMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

//This makes it easier to make a path with just a parralel command group
public abstract class VortechsPathWithCommandGroup implements VortechsPath{
    

    protected double distanceToTarget = Double.MAX_VALUE;

    
    @Override
    public void tick(Pose2d robotPose) {
        distanceToTarget = robotPose.getTranslation().getDistance(getTargetPose().getTranslation());
    }

    

    @Override
    public Command getParallelCommandGroup() {

        TreeMap<Double, Command> secondaryCommandMap = getCommands();

        Command[] commandArr = new Command[secondaryCommandMap.size()];

        // adds all the secondary commands to the array
        int index = 0;

        for (double distance : secondaryCommandMap.keySet()) {
            commandArr[index] =
                secondaryCommandMap
                    .get(distance)
                    .beforeStarting(new WaitUntilCommand(() -> this.distanceToTarget < distance));

            index++;
        }

        return new ParallelCommandGroup(commandArr);
    } 
    

    public abstract TreeMap<Double, Command> getCommands();




}
