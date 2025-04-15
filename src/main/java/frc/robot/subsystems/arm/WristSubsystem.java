package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    /*
	 * Motor
	 */
    private final SparkMax wristMotor = new SparkMax(Constants.ArmConstants.Wrist.ID, MotorType.kBrushless);
}
