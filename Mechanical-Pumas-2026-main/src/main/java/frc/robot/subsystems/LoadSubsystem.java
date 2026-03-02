package frc.robot.subsystems;

import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANIdConstants;

/**
 * LoadSubsystem - Controls a loader mechanism using a SparkMax motor controller.
 * 
 */
public class LoadSubsystem extends CancelableSubsystemBase {


    /**
     * Loader motor controller.
     * 
     */
    private final SparkMax loaderMotor;

    /**
     * Initialize the Load Subsystem.
     * 
     */
    public LoadSubsystem() {
        this.setName("LoadSubsystem");

        this.loaderMotor = new SparkMax(
                CANIdConstants.LOADER_ID,
                MotorType.kBrushless);

        SparkMaxConfig loaderConfig = new SparkMaxConfig();

        // We may want to switch to brake mode
        loaderConfig.idleMode(IdleMode.kCoast);

        loaderConfig.inverted(true);

        // 40 Amps and 12 volts are safe settings for most mechanisms.
        loaderConfig.smartCurrentLimit(40);
        loaderConfig.voltageCompensation(12.0);

        // Configure the SparkMax to use the config.
        // ResetMode.kResetSafeParameters makes sure unsafe params are reset.
        loaderMotor.configure(
                loaderConfig,
                ResetMode.kResetSafeParameters,
                null);
    }

    /**
     * Set the loader motor speed.
     * 
     * @param speed
     *              Speed (-1 to 1) to set the loader motor at.
     * 
     */
    public void setLoaderSpeed(double speed) {
        this.loaderMotor.set(speed);
    }

    /**
     * Cancel function to stop the loader motor
     */
    public void cancel() {
        this.loaderMotor.set(0);
    }
}