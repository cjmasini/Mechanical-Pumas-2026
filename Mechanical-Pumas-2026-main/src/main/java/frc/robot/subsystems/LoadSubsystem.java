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
    private SparkMax leftLoaderMotor;
    private SparkMax rightLoaderMotor;
    private SparkMax conveyorMotor;


    /**
     * Initialize the Load Subsystem.
     * 
     */
    public LoadSubsystem() {
        this.setName("LoadSubsystem");
        prepareLoadMotors();
    }

        private void prepareLoadMotors() {
        
        this.leftLoaderMotor = new SparkMax(
                CANIdConstants.LEFT_LOADER_ID,
                MotorType.kBrushless);

        this.rightLoaderMotor = new SparkMax(
                CANIdConstants.RIGHT_LOADER_ID,
                MotorType.kBrushless);

        this.conveyorMotor = new SparkMax(
                CANIdConstants.CONVEYOR_MOTOR_ID,
                MotorType.kBrushless);
        
        SparkMaxConfig leftLoaderConfig = new SparkMaxConfig();
        SparkMaxConfig rightLoaderConfig = new SparkMaxConfig();
        SparkMaxConfig conveyorConfig = new SparkMaxConfig();

        // We may want to switch to brake mode
        leftLoaderConfig.idleMode(IdleMode.kCoast);

        leftLoaderConfig.inverted(true);

        rightLoaderConfig.idleMode(IdleMode.kCoast);

        rightLoaderConfig.inverted(true);

        conveyorConfig.idleMode(IdleMode.kCoast);

        conveyorConfig.inverted(true);

        // 40 Amps and 12 volts are safe settings for most mechanisms.
        leftLoaderConfig.smartCurrentLimit(40);
        leftLoaderConfig.voltageCompensation(12.0);

        rightLoaderConfig.smartCurrentLimit(40);
        rightLoaderConfig.voltageCompensation(12.0);

        conveyorConfig.smartCurrentLimit(40);
        conveyorConfig.voltageCompensation(12.0);

        // Configure the SparkMax to use the config.
        // ResetMode.kResetSafeParameters makes sure unsafe params are reset.
        leftLoaderMotor.configure(
                leftLoaderConfig,
                ResetMode.kResetSafeParameters,
                null);
        rightLoaderMotor.configure(
                rightLoaderConfig,
                ResetMode.kResetSafeParameters,
                null);
        conveyorMotor.configure(
                conveyorConfig,
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
        this.leftLoaderMotor.set(speed);
        this.rightLoaderMotor.set(speed);
        this.conveyorMotor.set(speed);
    }

    /**
     * Cancel function to stop the loader motor
     */
    public void cancel() {
        this.leftLoaderMotor.set(0);
        this.rightLoaderMotor.set(0);
        this.conveyorMotor.set(0);
    }
}