package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimbSubsystem extends SubsystemBase {
    public final VictorSPX climbWinchMotor = new WPI_VictorSPX(Constants.ClimbConstants.ID);
    
    public ClimbSubsystem() {
        System.out.println("Created ClimbSubsystem");
    }

    public void setClimbWinchSpeed(double climbWinchSpeed) {
        climbWinchMotor.set(VictorSPXControlMode.PercentOutput, climbWinchSpeed);
    }

    @Override
    public void periodic() {
        
    }

    public Command climbUp(double climbUpWinchSpeed) {
        return new StartEndCommand(() -> setClimbWinchSpeed(climbUpWinchSpeed), () -> setClimbWinchSpeed(0), this);
    }
}
