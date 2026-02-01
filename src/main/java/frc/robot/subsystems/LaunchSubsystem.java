package frc.robot.subsystems;

import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANIdConstants;

/**
 * LaunchSubsystem - Controls a launcher mechanism using a SparkMax motor controller.
 * 
 */
public class LaunchSubsystem extends CancelableSubsystemBase {


    /**
     * Launcher motor controller.
     * 
     */
    private final SparkMax launcherMotor;

    /**
     * Initialize the Launch Subsystem.
     * 
     */
    public LaunchSubsystem() {
        this.setName("LaunchSubsystem");

        this.launcherMotor = new SparkMax(
                CANIdConstants.LAUNCHER_ID,
                MotorType.kBrushless);

        SparkMaxConfig launcherConfig = new SparkMaxConfig();

        // We may want to switch to brake mode
        launcherConfig.idleMode(IdleMode.kCoast);

        launcherConfig.inverted(true);

        // 40 Amps and 12 volts are safe settings for most mechanisms.
        launcherConfig.smartCurrentLimit(40);
        launcherConfig.voltageCompensation(12.0);

        // Configure the SparkMax to use the config.
        // ResetMode.kResetSafeParameters makes sure unsafe params are reset.
        launcherMotor.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                null);
    }

    /**
     * Set the launcher motor speed.
     * 
     * @param speed
     *              Speed (-1 to 1) to set the launcher motor at.
     * 
     */
    public void setLauncherSpeed(double speed) {
        this.launcherMotor.set(speed);
    }

    /**
     * Cancel function to stop the launcher motor
     */
    public void cancel() {
        this.launcherMotor.set(0);
    }
}