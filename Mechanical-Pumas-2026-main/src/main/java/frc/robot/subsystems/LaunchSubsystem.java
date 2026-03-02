package frc.robot.subsystems;

import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANIdConstants;

/**
 * LaunchSubsystem - Controls a launcher mechanism using two SparkFlex motor controllers and a loader mechanism using two SparkMax motor controllers.
 * 
 */
public class LaunchSubsystem extends CancelableSubsystemBase {


    /**
     * Launcher motor controllers.
     * 
     */
    private SparkFlex leftLauncherMotor;
    private SparkFlex rightLauncherMotor;

    /**
     * Loader motor controllers.
     * 
     */
    private SparkMax leftLoaderMotor;
    private SparkMax rightLoaderMotor;

    /**
     * Initialize the Launch Subsystem.
     * 
     */
    public LaunchSubsystem() {
        this.setName("LaunchSubsystem");
        prepareLaunchMotors();
        prepareLoadMotors();
    }

    /**
     * Set the speed for all motors.
     * 
     * @param speed
     *              Speed (-1 to 1) to set the motors at.
     * 
     */
    public void setLauncherSpeed(double speed) {
        this.leftLauncherMotor.set(speed);
        this.rightLauncherMotor.set(speed);
    }

    public void setLoaderSpeed(double speed) {
        this.leftLoaderMotor.set(speed);
        this.rightLoaderMotor.set(speed);
    }

    /**
     * Cancel function to stop the launcher motor
     */
    public void cancel() {
        this.leftLauncherMotor.set(0);
        this.rightLauncherMotor.set(0);
        this.leftLoaderMotor.set(0);
        this.rightLoaderMotor.set(0);
    }

    private void prepareLaunchMotors() {
         this.leftLauncherMotor = new SparkFlex(
                CANIdConstants.LEFT_LAUNCHER_ID,
                MotorType.kBrushless);

        this.rightLauncherMotor = new SparkFlex(
                CANIdConstants.RIGHT_LAUNCHER_ID,
                MotorType.kBrushless);

        SparkFlexConfig launcherConfig = new SparkFlexConfig();

        // We may want to switch to brake mode
        launcherConfig.idleMode(IdleMode.kCoast);

        launcherConfig.inverted(true);

        // 40 Amps and 12 volts are safe settings for most mechanisms.
        launcherConfig.smartCurrentLimit(40);
        launcherConfig.voltageCompensation(12.0);

        // Configure the SparkFlex to use the config.
        // ResetMode.kResetSafeParameters makes sure unsafe params are reset.
        leftLauncherMotor.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                null);

        rightLauncherMotor.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                null);
    }

    private void prepareLoadMotors() {
        
        this.leftLoaderMotor = new SparkMax(
                CANIdConstants.LEFT_LOADER_ID,
                MotorType.kBrushless);

        this.rightLoaderMotor = new SparkMax(
                CANIdConstants.RIGHT_LOADER_ID,
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
        leftLoaderMotor.configure(
                loaderConfig,
                ResetMode.kResetSafeParameters,
                null);

        rightLoaderMotor.configure(
                loaderConfig,
                ResetMode.kResetSafeParameters,
                null);
    }
}