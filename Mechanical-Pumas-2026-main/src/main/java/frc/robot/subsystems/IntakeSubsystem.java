package frc.robot.subsystems;

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
     * Intake motor controller.
     * 
     */
    private final SparkMax leaderDeployMotor;
    private final SparkMax followerDeployMotor;
    private final SparkMax rollerMotor;

    /**
     * Initialize the Intake Subsystem.
     * 
     */
    public IntakeSubsystem() {
        this.setName("IntakeSubsystem");

        this.leaderDeployMotor = new SparkMax(
                CANIdConstants.LEADER_DEPLOY_ID,
                MotorType.kBrushless);

        this.followerDeployMotor = new SparkMax(
                CANIdConstants.FOLLOWER_DEPLOY_ID,
                MotorType.kBrushless);

        this.rollerMotor = new SparkMax(
                CANIdConstants.ROLLER_ID,
                MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();

        // We may want to switch to brake mode
        intakeConfig.idleMode(IdleMode.kCoast);

        intakeConfig.inverted(true);

        // 40 Amps and 12 volts are safe settings for most mechanisms.
        intakeConfig.smartCurrentLimit(40);
        intakeConfig.voltageCompensation(12.0);

        // Configure the SparkMax to use the config.
        // ResetMode.kResetSafeParameters makes sure unsafe params are reset.
        leaderDeployMotor.configure(
                intakeConfig,
                ResetMode.kResetSafeParameters,
                null);

        followerDeployMotor.configure(
                intakeConfig,
                ResetMode.kResetSafeParameters,
                null);

        rollerMotor.configure(
                intakeConfig,
                ResetMode.kResetSafeParameters,
                null);
    }

    /**
     * Set the intake motor speed.
     * 
     * @param speed
     *              Speed (-1 to 1) to set the intake motor at.
     * 
     */
    public void setIntakeSpeed(double speed) {
        this.leaderDeployMotor.set(speed);
        this.followerDeployMotor.set(speed);
        this.rollerMotor.set(speed);
    }

    /**
     * Cancel function to stop the intake motor
     */
    public void cancel() {
        this.leaderDeployMotor.set(0);
        this.followerDeployMotor.set(0);
        this.rollerMotor.set(0);
    }
}