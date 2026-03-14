package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANIdConstants;

/**
 * IntakeSubsystem - Controls an intake mechanism using a SparkMax motor controller.
 * 
 */
public class IntakeSubsystem extends CancelableSubsystemBase {


    /**
     * Intake motor controllers.
     * 
     */
    private final SparkMax rollerMotor;

    /**
     * Initialize the Intake Subsystem.
     * 
     */
    public IntakeSubsystem() {
        this.setName("IntakeSubsystem");

        this.rollerMotor = new SparkMax(
                CANIdConstants.ROLLER_ID,
                MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        // We may want to switch to coast mode
        intakeConfig.idleMode(IdleMode.kBrake);

        intakeConfig.inverted(true);

        // 40 Amps and 12 volts are safe settings for most mechanisms.
        intakeConfig.smartCurrentLimit(30);
        intakeConfig.voltageCompensation(12.0);


        rollerMotor.configure(
                intakeConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /**
     * Set the intake motor speed.
     * 
     * @param speed
     *              Speed (-1 to 1) to set the intake motor at.
     * 
     */
    public void setIntakeSpeed(double speed) {
        this.rollerMotor.set(speed);
    }

    /**
     * Cancel function to stop the intake roller
     */
    public void cancel() {
        this.rollerMotor.set(0);
    }
}