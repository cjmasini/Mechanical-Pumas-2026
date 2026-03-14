package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIdConstants;

/**
 * LaunchSubsystem - Controls a launcher mechanism using two SparkFlex motor controllers and a loader mechanism using two SparkMax motor controllers.
 * Supports both manual speed control and velocity-based control with distance interpolation.
 */
public class LaunchSubsystem extends CancelableSubsystemBase {

    private SparkFlex leftLauncherMotor;
    private SparkFlex rightLauncherMotor;

    private SparkClosedLoopController leftClosedLoopController;
    private SparkClosedLoopController rightClosedLoopController;
    private RelativeEncoder leftLauncherEncoder;
    private RelativeEncoder rightLauncherEncoder;

    /** Maps distance (meters) to target RPM for auto-shooting */
    private final InterpolatingDoubleTreeMap distanceToRPMMap = new InterpolatingDoubleTreeMap();

    /** Minimum calibrated distance in meters */
    private double minDistance = Double.MAX_VALUE;
    /** Maximum calibrated distance in meters */
    private double maxDistance = Double.MIN_VALUE;

    // Default PID/FF values for tuning
    private static final double DEFAULT_P = 0.0001;
    private static final double DEFAULT_I = 0.0;
    private static final double DEFAULT_D = 0.0;
    private static final double DEFAULT_FF = 0.000175;

    private double targetVelocity = 0.0;

    /**
     * Initialize the Launch Subsystem.
     */
    public LaunchSubsystem() {
        this.setName("LaunchSubsystem");
        prepareLaunchMotors();
        initSmartDashboard();
    }

    private void initSmartDashboard() {
        SmartDashboard.putNumber("Launcher/P", DEFAULT_P);
        SmartDashboard.putNumber("Launcher/I", DEFAULT_I);
        SmartDashboard.putNumber("Launcher/D", DEFAULT_D);
        SmartDashboard.putNumber("Launcher/FF", DEFAULT_FF);
    }

    @Override
    public void periodic() {
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

    private void prepareLaunchMotors() {
         this.leftLauncherMotor = new SparkFlex(
                CANIdConstants.LEFT_LAUNCHER_ID,
                MotorType.kBrushless);

        this.rightLauncherMotor = new SparkFlex(
                CANIdConstants.RIGHT_LAUNCHER_ID,
                MotorType.kBrushless);

        SparkFlexConfig launcherConfig = new SparkFlexConfig();
    
        // We may want to switch to brake mode
        launcherConfig.idleMode(IdleMode.kBrake);
        launcherConfig.inverted(false);

        launcherConfig.smartCurrentLimit(40);
        launcherConfig.voltageCompensation(12.0);
    
        launcherConfig.closedLoop
            .pid(0.0001, 0.0, 0.0)
            .outputRange(-1.0, 1.0)
            .feedForward.kV(0.000175);
        launcherConfig.inverted(true);

        leftLauncherMotor.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                null);



        rightLauncherMotor.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                null);
    
    
        this.leftClosedLoopController = leftLauncherMotor.getClosedLoopController();
        this.leftLauncherEncoder = leftLauncherMotor.getEncoder();
        this.rightClosedLoopController = rightLauncherMotor.getClosedLoopController();
        this.rightLauncherEncoder = rightLauncherMotor.getEncoder();

        createDistanceMap();
    }


    private void createDistanceMap() {
        addCalibrationPoint(1.0, 2000.0);
        addCalibrationPoint(2.0, 2500.0);
        addCalibrationPoint(3.0, 3000.0);
        addCalibrationPoint(4.0, 3500.0);
        addCalibrationPoint(5.0, 4000.0);
        addCalibrationPoint(6.0, 4500.0);
    }

    /**
     *      * Adds a calibration point to the distance-to-RPM map and updates min/max bounds.
     *
     * @param distanceMeters Distance to target in meters
     * @param rpm            Target RPM for this distance
     */

    private void addCalibrationPoint(double distanceMeters, double rpm) {
        distanceToRPMMap.put(distanceMeters, rpm);
        minDistance = Math.min(minDistance, distanceMeters);
        maxDistance = Math.max(maxDistance, distanceMeters);
    }

    /**
     * Set the launcher motor speed 
     *
     * @param speed Speed (-1 to 1) to set the launcher motor at.
     */
    public void setLauncherPower(double speed) {
        this.leftLauncherMotor.set(speed);
        this.rightLauncherMotor.set(-speed);
    }
    /**
     * Set the launcher motor to a target velocity
     *
     * @param rpm Target velocity in RPM
     */
    public void setLauncherVelocity(double rpm) {
        this.targetVelocity = rpm;
        leftClosedLoopController.setSetpoint(rpm, ControlType.kVelocity);
        rightClosedLoopController.setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * Sets the launcher velocity based on the distance to target.
     * Uses linear interpolation between calibrated distance/RPM pairs.
     *
     * @param distanceMeters Distance to target in meters
     */
    public void setLauncherVelocityForDistance(double distanceMeters) {
        double targetRPM = distanceToRPMMap.get(distanceMeters);
        setLauncherVelocity(targetRPM);
    }

    /**
     * Gets the current launcher velocity from the encoder.
     *
     * @return Current velocity in RPM (average of both motors)
     */
    public double getLauncherVelocity() {
        return (leftLauncherEncoder.getVelocity() + rightLauncherEncoder.getVelocity()) / 2.0;
    }

    /**
     * Gets the target velocity.
     *
     * @return Target velocity in RPM
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Checks if the launcher is at the target velocity within a tolerance.
     *
     * @param toleranceRPM Acceptable error in RPM
     * @return true if at target velocity
     */
    public boolean isAtTargetVelocity(double toleranceRPM) {
        return Math.abs(getLauncherVelocity() - targetVelocity) < toleranceRPM;
    }

    /**
     * Checks if the launcher is at the target velocity with default 100 RPM tolerance.
     *
     * @return true if at target velocity
     */
    public boolean isAtTargetVelocity() {
        return isAtTargetVelocity(100.0);
    }

    /**
     * Gets the minimum calibrated distance.
     *
     * @return Minimum distance in meters from the calibration table
     */
    public double getMinDistance() {
        return minDistance;
    }

    /**
     * Gets the maximum calibrated distance.
     *
     * @return Maximum distance in meters from the calibration table
     */
    public double getMaxDistance() {
        return maxDistance;
    }

    /**
     * Cancel function to stop the launcher motor.
     */
    @Override
    public void cancel() {
        this.leftLauncherMotor.set(0);
        this.rightLauncherMotor.set(0);
    }
}
