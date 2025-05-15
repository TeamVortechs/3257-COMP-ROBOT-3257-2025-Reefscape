package frc.robot.util.FieldMovement.vortechsPath;

import java.util.TreeMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

//This makes it easier to make a path with a parralel command group that is affected by distance
public abstract class VortechsPathWithCommandGroup extends VortechsPath{

    //abstract super constructor
    public VortechsPathWithCommandGroup(Supplier<Pose2d> robotPoseSupplier) {
        super(robotPoseSupplier);
    }
    
    //builds teh command group that only calls commands when the robot is close enough
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
                    .beforeStarting(new WaitUntilCommand(() -> getDistance() < distance));

            index++;
        }

        return new ParallelCommandGroup(commandArr);
    } 
    

    //the parralel command group
    public abstract TreeMap<Double, Command> getCommands();




}
