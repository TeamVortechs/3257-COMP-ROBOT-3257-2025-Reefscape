package frc.robot.util.simulation;

import java.time.chrono.HijrahChronology;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.photonvision.estimation.RotTrlTransform3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


public class VisualSimulator {

    private final LoggedMechanism2d panel;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d ligament;

    private Translation2d startPoint;

    private DoubleSupplier height;
    private DoubleSupplier angle;

    VisualSimulator(Translation2d startPoint, DoubleSupplier angle, DoubleSupplier height, double width, Color8Bit color, String name) {

        this.startPoint = new Translation2d(startPoint.getX(), startPoint.getY());

        this.panel = new LoggedMechanism2d(width, height.getAsDouble());

        this.root =
            panel.getRoot(name, startPoint.getX(), startPoint.getY());

        this.ligament =
            root.append(
                new LoggedMechanismLigament2d(
                    name, Units.inchesToMeters(25), 90, 6, new Color8Bit(Color.kYellow)));

        this.height = height;
        this.angle = angle;
    }

    public void periodic() {
        ligament.setAngle(angle.getAsDouble());
        ligament.setLength(height.getAsDouble());
    }

    //changing the position of a root node
    public void setStartPoint(Translation2d startPoint) {
        this.startPoint = startPoint;
    
        root.setPosition(startPoint.getX(), startPoint.getY());
    }

    public void setX(double x) {
        setStartPoint(new Translation2d(x, startPoint.getY()));
    }

    public void setY(double y) {
        setStartPoint(new Translation2d(startPoint.getX(), y));
    }

    public Translation2d getStartPoint() {
        return startPoint;
    }

    public double getAngle() {
        return angle.getAsDouble();
    }

    public double getHeight() {
        return height.getAsDouble();
    }



    //sets the color
    public void setColor(Color8Bit color) {
        ligament.setColor(color);
    }

}
