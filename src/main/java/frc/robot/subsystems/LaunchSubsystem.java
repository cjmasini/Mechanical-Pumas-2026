package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.CANIdConstants;

/**
 * LaunchSubsystem - Controls a launcher mechanism using a SparkMax motor controller.
 * Supports both manual speed control and velocity-based control with distance interpolation.
 */
public class LaunchSubsystem extends CancelableSubsystemBase {

    private final SparkMax launcherMotor;
    private final SparkClosedLoopController closedLoopController;
    private final RelativeEncoder launcherEncoder;

    /** Maps distance (meters) to target RPM for auto-shooting */
    private final InterpolatingDoubleTreeMap distanceToRPMMap = new InterpolatingDoubleTreeMap();

    /** Minimum calibrated distance in meters */
    private double minDistance = Double.MAX_VALUE;
    /** Maximum calibrated distance in meters */
    private double maxDistance = Double.MIN_VALUE;

    /**
     * Initialize the Launch Subsystem.
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

        launcherConfig.smartCurrentLimit(40);
        launcherConfig.voltageCompensation(12.0);

        ClosedLoopConfig closedLoopConfig = launcherConfig.closedLoop;
        closedLoopConfig.pidf(
                0.0001,   // P - tune on robot
                0.0,      // I
                0.0,      // D
                0.000175  // F (feedforward) - approximate: 1/max_rpm
        );
        closedLoopConfig.outputRange(-1.0, 1.0);

        launcherMotor.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                null);

        this.closedLoopController = launcherMotor.getClosedLoopController();
        this.launcherEncoder = launcherMotor.getEncoder();

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
     * Adds a calibration point to the distance-to-RPM map and updates min/max bounds.
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
        this.launcherMotor.set(speed);
    }

    /**
     * Set the launcher motor to a target velocity
     *
     * @param rpm Target velocity in RPM
     */
    public void setLauncherVelocity(double rpm) {
        closedLoopController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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
     * @return Current velocity in RPM
     */
    public double getLauncherVelocity() {
        return launcherEncoder.getVelocity();
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
        this.launcherMotor.set(0);
    }
}
