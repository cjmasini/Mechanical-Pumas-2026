package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANIdConstants;

/**
 * LoadSubsystem - Controls loaders and indexer
 */
public class LoadSubsystem extends CancelableSubsystemBase {

    private SparkFlex indexerMotor;
    private SparkMax leftLoaderMotor;
    private SparkMax rightLoaderMotor;

    /**
     * Initialize the Load Subsystem.
     */
    public LoadSubsystem() {
        this.setName("LoadSubsystem");
        prepareLoadMotors();
    }

    @Override
    public void periodic() {
    }

    private void prepareLoadMotors() {
        this.indexerMotor = new SparkFlex(
                CANIdConstants.INDEXER_MOTOR_ID,
                MotorType.kBrushless);

        this.leftLoaderMotor = new SparkMax(
                CANIdConstants.LEFT_LOADER_ID,
                MotorType.kBrushless);

        this.rightLoaderMotor = new SparkMax(
                CANIdConstants.RIGHT_LOADER_ID,
                MotorType.kBrushless);

        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        SparkMaxConfig loaderConfig = new SparkMaxConfig();

        indexerConfig.idleMode(IdleMode.kBrake);
        indexerConfig.inverted(false);
        indexerConfig.smartCurrentLimit(30);
        indexerConfig.voltageCompensation(12.0);

        loaderConfig.idleMode(IdleMode.kBrake);
        loaderConfig.inverted(false);
        loaderConfig.smartCurrentLimit(40);
        loaderConfig.voltageCompensation(12.0);

        indexerMotor.configure(
                indexerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        rightLoaderMotor.configure(
                loaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        loaderConfig.inverted(true);
        leftLoaderMotor.configure(
                loaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    /**
     * Set the conveyor motor speed.
     *
     * @param speed Speed (-1 to 1) to set the conveyor motor at.
     */
    public void setIndexerSpeed(double speed) {
        this.indexerMotor.set(speed);
    }

    /**
     * Set the loader motor speed.
     *
     * @param speed Speed (-1 to 1) to set the loader motors at.
     */
    public void setLoaderSpeed(double speed) {
        this.leftLoaderMotor.set(speed);
        this.rightLoaderMotor.set(speed);
    }

    /**
     * Cancel function to stop the loader motors.
     */
    public void cancel() {
        this.indexerMotor.set(0);
        this.leftLoaderMotor.set(0);
        this.rightLoaderMotor.set(0);
    }
}
