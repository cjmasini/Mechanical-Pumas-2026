package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIdConstants;

/**
 * IntakeSubsystem - Controls the intake roller and deploy mechanism.
 */
public class IntakeSubsystem extends CancelableSubsystemBase {

    // Intake position constants
    private static final double STOWED_POSITION = 0.0;
    private static final double DEPLOYED_POSITION = -11.4;
    private static final double BUCK_POSITION = -1.5;
    private static final double POSITION_TOLERANCE = 0.1;

    // Current monitoring constants
    private static final int STALL_CURRENT_THRESHOLD = 40;

    // TODO: Tune these
    private static final double INTAKE_P = 0.05;
    private static final double INTAKE_I = 0.0;
    private static final double INTAKE_D = 0.02;

    private final SparkFlex rollerMotor;
    private SparkMax leaderDeployMotor;
    private SparkMax followerDeployMotor;

    private RelativeEncoder deployEncoder;
    private SparkClosedLoopController deployPID;

    /**
     * Initialize the Intake Subsystem.
     */
    public IntakeSubsystem() {
        this.setName("IntakeSubsystem");

        this.rollerMotor = new SparkFlex(
                CANIdConstants.ROLLER_ID,
                MotorType.kBrushless);

        SparkFlexConfig intakeConfig = new SparkFlexConfig();

        // We may want to switch to coast mode
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeConfig.inverted(true);

        // 40 Amps and 12 volts are safe settings for most mechanisms.
        intakeConfig.smartCurrentLimit(30);
        intakeConfig.voltageCompensation(12.0);

        rollerMotor.configure(
                intakeConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        prepareDeployMotors();
        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putNumber("Deploy/P", INTAKE_P);
        SmartDashboard.putNumber("Deploy/I", INTAKE_I);
        SmartDashboard.putNumber("Deploy/D", INTAKE_D);
        SmartDashboard.putNumber("Deploy/Setpoint", STOWED_POSITION);
    }

    private void prepareDeployMotors() {
        this.leaderDeployMotor = new SparkMax(
                CANIdConstants.LEADER_DEPLOY_ID,
                MotorType.kBrushless);

        this.followerDeployMotor = new SparkMax(
                CANIdConstants.FOLLOWER_DEPLOY_ID,
                MotorType.kBrushless);

        SparkMaxConfig leaderDeployConfig = new SparkMaxConfig();
        SparkMaxConfig followerDeployConfig = new SparkMaxConfig();

        leaderDeployConfig.idleMode(IdleMode.kBrake);
        leaderDeployConfig.smartCurrentLimit(STALL_CURRENT_THRESHOLD);
        leaderDeployConfig.voltageCompensation(12.0);
        leaderDeployConfig.closedLoop
            .p(INTAKE_P)
            .i(INTAKE_I)
            .d(INTAKE_D);

        followerDeployConfig.idleMode(IdleMode.kBrake);
        followerDeployConfig.smartCurrentLimit(STALL_CURRENT_THRESHOLD);
        followerDeployConfig.follow(CANIdConstants.LEADER_DEPLOY_ID, true);

        leaderDeployMotor.configure(
                leaderDeployConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        followerDeployMotor.configure(
                followerDeployConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        this.deployEncoder = leaderDeployMotor.getEncoder();
        this.deployPID = leaderDeployMotor.getClosedLoopController();
    }

    /**
     * Set the intake motor speed.
     *
     * @param speed Speed (-1 to 1) to set the intake motor at.
     */
    public void setIntakeSpeed(double speed) {
        this.rollerMotor.set(speed);
    }

    /**
     * Get the editable setpoint from SmartDashboard.
     *
     * @return The target position from SmartDashboard
     */
    public double getDashboardSetpoint() {
        return SmartDashboard.getNumber("Deploy/Setpoint", STOWED_POSITION);
    }

    /**
     * Move intake to the SmartDashboard setpoint. Call this periodically until it returns true.
     *
     * @return true if at position or stalled, false if still moving
     */
    public boolean goToSetpoint() {
        return setIntakePosition(-5);
    }

    /**
     * Deploy the intake. Call this periodically until it returns true.
     *
     * @return true if intake is deployed or stalled, false if still moving
     */
    public boolean deployIntake() {
        return setIntakePosition(DEPLOYED_POSITION);
    }

    /**
     * Stow the intake. Call this periodically until it returns true.
     *
     * @return true if intake is stowed or stalled, false if still moving
     */
    public boolean stowIntake() {
        return setIntakePosition(STOWED_POSITION);
    }

    /**
     * Move intake to buck position (slightly open). Call this periodically until it returns true.
     *
     * @return true if at buck position or stalled, false if still moving
     */
    public boolean buckIntake() {
        return setIntakePosition(BUCK_POSITION);
    }

    /**
     * Move intake to a target position. Call this periodically until it returns true.
     *
     * @param targetPosition target position in motor rotations
     * @return true if at position or stalled, false if still moving
     */
    public boolean setIntakePosition(double targetPosition) {
        double currentPosition = deployEncoder.getPosition();
        double error = targetPosition - currentPosition;

        if (Math.abs(error) < POSITION_TOLERANCE) {
            return true;
        }

        // Use PID controller to move toward target position
        deployPID.setSetpoint(targetPosition, ControlType.kPosition);

        return false;
    }

    /**
     * Check if the intake is deployed
     *
     * @return current position
     */
    public boolean isDeployed() {
        return Math.abs(DEPLOYED_POSITION - deployEncoder.getPosition()) < POSITION_TOLERANCE;
    }

    /**
     * Get the current intake position in motor rotations.
     *
     * @return current position
     */
    public double getIntakePosition() {
        return deployEncoder.getPosition();
    }

    /**
     * Cancel function to stop the intake roller and deploy motors.
     */
    public void cancel() {
        this.rollerMotor.set(0);
        this.leaderDeployMotor.set(0);
    }
}
