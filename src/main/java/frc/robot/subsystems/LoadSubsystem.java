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
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIdConstants;

/**
 * LoadSubsystem - Controls a loader mechanism using a SparkMax motor controller.
 * */
public class LoadSubsystem extends CancelableSubsystemBase {

    // Intake position constants (in motor rotations)
    private static final double STOWED_POSITION = 0.0;
    private static final double DEPLOYED_POSITION = -11.4; // TODO: Tune this value
    private static final double BUCK_POSITION = -1; // Slightly open for bucking
    private static final double POSITION_TOLERANCE = 0.2;

    // Current monitoring constants
    private static final double STALL_CURRENT_THRESHOLD = 40.0;

    // TODO: Tune these
    private static final double INTAKE_P = 0.05; //Increase if slow
    private static final double INTAKE_I = 0.0;
    private static final double INTAKE_D = 0.02; //Increase if overshooting

    private SparkMax leaderDeployMotor;
    private SparkMax followerDeployMotor;
    private SparkFlex indexerMotor;
    private SparkMax leftLoaderMotor;
    private SparkMax rightLoaderMotor;

    private RelativeEncoder deployEncoder;
    private SparkClosedLoopController deployPID;


    /**
     * Initialize the Load Subsystem.
     * */
    public LoadSubsystem() {
        this.setName("LoadSubsystem");
        prepareLoadMotors();
        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putNumber("Deploy/P", INTAKE_P);
        SmartDashboard.putNumber("Deploy/I", INTAKE_I);
        SmartDashboard.putNumber("Deploy/D", INTAKE_D);
        SmartDashboard.putNumber("Deploy/Setpoint", STOWED_POSITION);
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

    @Override
    public void periodic() {
        // Publish debug info
        SmartDashboard.putNumber("Deploy/Position", deployEncoder.getPosition());
        SmartDashboard.putNumber("Deploy/Current", leaderDeployMotor.getOutputCurrent());
        SmartDashboard.putNumber("Deploy/Velocity", deployEncoder.getVelocity());
        SmartDashboard.putNumber("Loader/RightCurrent", rightLoaderMotor.getOutputCurrent());
        SmartDashboard.putNumber("Loader/RightOutput", rightLoaderMotor.getAppliedOutput());
        SmartDashboard.putString("Loader/RightCANError", rightLoaderMotor.getLastError().toString());
        // SmartDashboard.putNumber("Loader/LeftCurrent", leftLoaderMotor.getStatorCurrent());

        // // Read tunable PID values and update if changed
        // double newP = SmartDashboard.getNumber("Deploy/P", INTAKE_P);
        // double newI = SmartDashboard.getNumber("Deploy/I", INTAKE_I);
        // double newD = SmartDashboard.getNumber("Deploy/D", INTAKE_D);

        // SparkMaxConfig pidUpdate = new SparkMaxConfig();
        // pidUpdate.closedLoop.p(newP).i(newI).d(newD);
        // leaderDeployMotor.configure(pidUpdate, ResetMode.kNoResetSafeParameters, null);
    }

        private void prepareLoadMotors() {
        
        this.leaderDeployMotor = new SparkMax(
                CANIdConstants.LEADER_DEPLOY_ID,
                MotorType.kBrushless);

        this.followerDeployMotor = new SparkMax(
                CANIdConstants.FOLLOWER_DEPLOY_ID,
                MotorType.kBrushless);

        this.indexerMotor = new SparkFlex(
                CANIdConstants.INDEXER_MOTOR_ID,
                MotorType.kBrushless);

        this.leftLoaderMotor = new SparkMax(
                CANIdConstants.LEFT_LOADER_ID,
                MotorType.kBrushless);

        // this.leftLoaderMotor = new ThriftyNova(CANIdConstants.LEFT_LOADER_ID);
        this.rightLoaderMotor = new SparkMax(
                CANIdConstants.RIGHT_LOADER_ID,
                MotorType.kBrushless);

        SparkMaxConfig leaderDeployConfig = new SparkMaxConfig();
        SparkMaxConfig followerDeployConfig = new SparkMaxConfig();
        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        SparkMaxConfig loaderConfig = new SparkMaxConfig();

        leaderDeployConfig.idleMode(IdleMode.kBrake);
        leaderDeployConfig.smartCurrentLimit(10);
        leaderDeployConfig.voltageCompensation(12.0);
        leaderDeployConfig.closedLoop
            .p(INTAKE_P)
            .i(INTAKE_I)
            .d(INTAKE_D);

        followerDeployConfig.idleMode(IdleMode.kBrake);
        followerDeployConfig.follow(CANIdConstants.LEADER_DEPLOY_ID, true);

        indexerConfig.idleMode(IdleMode.kBrake);
        indexerConfig.inverted(false);
        indexerConfig.smartCurrentLimit(30);
        indexerConfig.voltageCompensation(12.0);

        loaderConfig.idleMode(IdleMode.kBrake);
        loaderConfig.inverted(false);
        loaderConfig.smartCurrentLimit(40);
        loaderConfig.voltageCompensation(12.0);

        leaderDeployMotor.configure(
                leaderDeployConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        followerDeployMotor.configure(
                followerDeployConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        indexerMotor.configure(
                indexerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        rightLoaderMotor.configure(
                loaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        // // Nova configuration to match the SparkMax loader settings
        // leftLoaderMotor(true); // Matches the config change made before the rightLoaderMotor configure call
        // leftLoaderMotor.setBrakeMode(true);
        // leftLoaderMotor.setMaxCurrent(CurrentType.STATOR,60);
        loaderConfig.inverted(true);
        leftLoaderMotor.configure(
                loaderConfig,
                ResetMode.kResetSafeParameters,
                null);
        this.deployEncoder = leaderDeployMotor.getEncoder();
        this.deployPID = leaderDeployMotor.getClosedLoopController();
    }

    /**
     * Set the conveyor motor speed.
     * * @param speed
     * Speed (-1 to 1) to set the conveyor motor at.
     * */
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
     * Deploy the intake. Call this periodically until it returns true.
     * Monitors current to stop if the intake stalls.
     *
     * @return true if intake is deployed or stalled, false if still moving
     */
    public boolean deployIntake() {
        return setIntakePosition(DEPLOYED_POSITION);
    }

    /**
     * Stow the intake. Call this periodically until it returns true.
     * Monitors current to stop if the intake stalls.
     *
     * @return true if intake is stowed or stalled, false if still moving
     */
    public boolean stowIntake() {
        return setIntakePosition(STOWED_POSITION);
    }

    /**
     * Move intake to buck position (slightly open). Call this periodically until it returns true.
     * Monitors current to stop if the intake stalls.
     *
     * @return true if at buck position or stalled, false if still moving
     */
    public boolean buckIntake() {
        return setIntakePosition(BUCK_POSITION);
    }

    /**
     * Move intake to a target position. Call this periodically until it returns true.
     * Monitors current to stop if the intake stalls.
     *
     * @param targetPosition target position in motor rotations
     * @return true if at position or stalled, false if still moving
     */
    public boolean setIntakePosition(double targetPosition) {
        double currentPosition = deployEncoder.getPosition();
        double error = targetPosition - currentPosition;

        if (Math.abs(error) < POSITION_TOLERANCE) {
            // leaderDeployMotor.set(0);
            return true;
        }

        // if (leaderDeployMotor.getOutputCurrent() > STALL_CURRENT_THRESHOLD) {
        //     leaderDeployMotor.set(0);
        //     SmartDashboard.putBoolean("Stalled", true);
        //     return true;
        // }

        // Use PID controller to move toward target position
        deployPID.setSetpoint(targetPosition, ControlType.kPosition);

        return false;
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
     * Rezero the intake by slowly moving downwards until a stall is detected.
     * When stalled, sets the encoder position to DEPLOYED_POSITION.
     * Call this periodically until it returns true.
     *
     * @return true if rezero is complete, false if still moving
     */
    public boolean rezeroIntake() {
        // if (leaderDeployMotor.getOutputCurrent() > STALL_CURRENT_THRESHOLD) {
        //     leaderDeployMotor.set(0);
        //     deployEncoder.setPosition(DEPLOYED_POSITION);
        //     return true;
        // }

        leaderDeployMotor.set(0.2);
        return false;
    }

    /**
     * Cancel function to stop the loader motors
     */
    public void cancel() {
        this.leaderDeployMotor.set(0);
        this.indexerMotor.set(0);
        this.leftLoaderMotor.set(0);
        this.rightLoaderMotor.set(0);
    }
}