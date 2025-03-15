package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Cradle {
    private SparkMax cradleMotor;

    public Cradle(int cradleMotorID) {
        cradleMotor = new SparkMax(cradleMotorID, MotorType.kBrushed);
    }

    public void runCradle(double power) {
        cradleMotor.set(power);
    }
}
