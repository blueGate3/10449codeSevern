package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SwerveModule extends SubsystemBase{
    private SparkMax m_turnMotor;
    private SparkMaxConfig m_turnMotorConfig;

    private SparkFlex m_driveMotor;
    private SparkFlexConfig m_driveMotorConfig;

    private SparkClosedLoopController turnController;
    
    public SwerveModule(int driveCANID, int turnCANID, boolean driveInverted) {
        m_driveMotor = new SparkFlex(driveCANID, MotorType.kBrushless);
        m_turnMotor = new SparkMax(turnCANID, MotorType.kBrushless);

        m_driveMotorConfig = new SparkFlexConfig();
        m_turnMotorConfig = new SparkMaxConfig();

        m_driveMotorConfig
        .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(.5)
            .inverted(false);

        m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_turnMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);

        m_turnMotorConfig.absoluteEncoder
            .positionConversionFactor((2*Math.PI))
            .inverted(true); //using rev througbore encoder

        m_turnMotorConfig.closedLoop
            .pid(.7, 0.0, 0.05) //todo will need new pid values, these are from our bot. 
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) //when using old method, primaryEncoder was the only thing that worked, absoluteEncoder should work tho.
            .positionWrappingEnabled(true) //this and line below it allow for position wrapping between 0 and 2pi radians 
            .positionWrappingInputRange(0, 2*Math.PI)
            .outputRange(-1, 1);


        m_turnMotor.configure(m_turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnController = m_turnMotor.getClosedLoopController();
        System.out.println("Turn Motor CAN ID: " + m_turnMotor.getEncoder().getPosition());

        //NOTE TO ANY PROGRAMMER: BECAUSE WE ARE ONLY RUNNING THIS IN TANK MODE, IT MAY BE BEST TO JUST SET THE TURNING MOTOR TO BRAKE MODE 
        // , ENSURE THE BRAKE MODE IS SET TO BRAKE MODE, AND KEEP WHEELS ALIGNED TO DRIVE TANK. YOU MAY ALSO WANT TO RESET ALL ENCODER VALUES 
        //WHEN THE CODE STARTS. IDK IVE NEVER DONE THIS SO FAST BEFORE IM SCARED
    }

    public void setDrivePower(double  power) {
        m_driveMotor.set(power);
    }

    public void setTurnPosition(double rots) {
        turnController.setReference(rots, ControlType.kPosition);
    }

}