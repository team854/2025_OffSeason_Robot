package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
	/*
	 * Constants
	 */
	private final double stage1MaxHeight = Constants.ElevatorConstants.Stage1.HARD_MAX_HEIGHT.in(Meter);
	private final double stage2MaxHeight = Constants.ElevatorConstants.Stage2.HARD_MAX_HEIGHT.in(Meter);
	private final double maxHeight = stage1MaxHeight + stage2MaxHeight;

	/*
	 * Motors
	 */
	private final SparkMax stage1Motor = new SparkMax(Constants.ElevatorConstants.Stage1.ID, MotorType.kBrushless);
	private final RelativeEncoder stage1Encoder = stage1Motor.getEncoder();

    private final SparkMax stage2Motor = new SparkMax(Constants.ElevatorConstants.Stage2.ID, MotorType.kBrushless);
    private final RelativeEncoder stage2Encoder = stage2Motor.getEncoder();
    private final AbsoluteEncoder stage2AbsoluteEncoder = stage2Motor.getAbsoluteEncoder();

	/*
	 * Simulation
	 */

	private final DCMotor stage1Gearbox = DCMotor.getNEO(1);
	private final SparkMaxSim stage1MotorSim = new SparkMaxSim(stage1Motor, stage1Gearbox);
	private ElevatorSim stage1ElevatorSim = null;

	private final DCMotor stage2Gearbox = DCMotor.getNEO(1);
	private final SparkMaxSim stage2MotorSim = new SparkMaxSim(stage2Motor, stage2Gearbox);
	private ElevatorSim stage2ElevatorSim = null;
}
