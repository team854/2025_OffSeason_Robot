package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.lang.reflect.Field;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * 
 * CoralFlightSim is a modified version of {@link ReefscapeCoralOnFly}
 */
public class CoralFlightSim extends ReefscapeCoralOnFly {
    
    public CoralFlightSim(GamePieceOnFieldSimulation.GamePieceInfo info,
            Translation3d initialPosition,
            Translation3d initialLaunchingVelocityMPS,
            Rotation3d gamePieceRotation) {
		// Give the super some dummy inputs
        super(new Translation2d(),
            new Translation2d(),
            new ChassisSpeeds(),
            new Rotation2d(),
            Meters.of(0),
            MetersPerSecond.of(0),
            Radians.of(0)
);
		
		// Written by claude 2.7 with a bit of cleanup
		try {
			// For each final field, use reflection to make it accessible and set the value
			Field infoField = GamePieceProjectile.class.getDeclaredField("info");
			infoField.setAccessible(true);
			infoField.set(this, info);
			
			Field gamePieceTypeField = GamePieceProjectile.class.getDeclaredField("gamePieceType");
			gamePieceTypeField.setAccessible(true);
			gamePieceTypeField.set(this, info.type());
			
			Field initialPositionField = GamePieceProjectile.class.getDeclaredField("initialPosition");
			initialPositionField.setAccessible(true);
			initialPositionField.set(this, initialPosition.toTranslation2d());
			
			Field initialLaunchingVelocityMPSField = GamePieceProjectile.class.getDeclaredField("initialLaunchingVelocityMPS");
			initialLaunchingVelocityMPSField.setAccessible(true);
			initialLaunchingVelocityMPSField.set(this, initialLaunchingVelocityMPS.toTranslation2d());
			
			Field initialHeightField = GamePieceProjectile.class.getDeclaredField("initialHeight");
			initialHeightField.setAccessible(true);
			initialHeightField.set(this, initialPosition.getZ());
			
			Field initialVerticalSpeedMPSField = GamePieceProjectile.class.getDeclaredField("initialVerticalSpeedMPS");
			initialVerticalSpeedMPSField.setAccessible(true);
			initialVerticalSpeedMPSField.set(this, initialLaunchingVelocityMPS.getZ());
			
			Field gamePieceRotationField = GamePieceProjectile.class.getDeclaredField("gamePieceRotation");
			gamePieceRotationField.setAccessible(true);
			gamePieceRotationField.set(this, gamePieceRotation);
			
			Field launchedTimerField = GamePieceProjectile.class.getDeclaredField("launchedTimer");
			launchedTimerField.setAccessible(true);
			launchedTimerField.set(this, new Timer());
			
		} catch (NoSuchFieldException | IllegalAccessException e) {
			throw new RuntimeException("Failed to set fields via reflection", e); 
		}

        super.enableBecomesGamePieceOnFieldAfterTouchGround();
        super.withTouchGroundHeight(0.2);

        // MAKE IT USE calculateInitialProjectileVelocityMPS TO ACCOUNT FOR THE ARMS / ROBOT SPEED
    }

    @Override
    public void addGamePieceAfterTouchGround(SimulatedArena simulatedArena) {
        if (!super.becomesGamePieceOnGroundAfterTouchGround) return;
        simulatedArena.addGamePiece(new GamePieceOnFieldSimulation(
                ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
                () -> Math.max(
                        ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO
                                        .gamePieceHeight()
                                        .in(Meters) / 2
                                ,
                        getPositionAtTime(super.launchedTimer.get()).getZ()),
                new Pose2d(
                        getPositionAtTime(launchedTimer.get()).toTranslation2d(), initialLaunchingVelocityMPS.getAngle()),
                super.initialLaunchingVelocityMPS));
    }
}
