package frc.robot.subsystems.simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulationSubsystem extends SubsystemBase{


    public SimulationSubsystem() {
        SimulatedArena.getInstance().resetFieldForAuto();

        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
            // We must specify a heading since the coral is a tube
            new Pose2d(2, 2, Rotation2d.fromDegrees(90))));

    }
}
