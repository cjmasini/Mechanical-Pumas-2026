package frc.robot.subsystems;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.CANIdConstants;

/**
 * LaunchSubsystem - Controls a launcher mechanism using two SparkFlex motor controllers and a loader mechanism using two SparkMax motor controllers.
 * Supports both manual speed control and velocity-based control with distance interpolation.
 */
public class LaunchSubsystem extends CancelableSubsystemBase {


    private SparkFlex leftLauncherMotor;
    private SparkFlex rightLauncherMotor;

    /**
     * Loader motor controllers.
     * 
     */
    private SparkMax leftLoaderMotor;
    private SparkMax rightLoaderMotor;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder launcherEncoder;

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
        prepareLaunchMotors();
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
        launcherConfig.idleMode(IdleMode.kCoast);
        launcherConfig.inverted(true);

        launcherConfig.smartCurrentLimit(40);
        launcherConfig.voltageCompensation(12.0);
    
        ClosedLoopConfig closedLoopConfig = launcherConfig.closedLoop;
         closedLoopConfig.pidf(0.0001, 0.0, 0.0, 0.000175);
        closedLoopConfig.outputRange(-1.0, 1.0);

        leftLauncherMotor.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                null);

        rightLauncherMotor.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                null);
    
    
        this.closedLoopController = leftLauncherMotor.getClosedLoopController();
        this.launcherEncoder = leftLauncherMotor.getEncoder();
        this.closedLoopController = rightLauncherMotor.getClosedLoopController();
        this.launcherEncoder = rightLauncherMotor.getEncoder();

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
        this.leftLauncherMotor.set(speed);
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
    /**
     * Cancel function to stop the launcher motor
     */
    public void cancel() {
        this.leftLauncherMotor.set(0);
        this.rightLauncherMotor.set(0);
        this.leftLoaderMotor.set(0);
        this.rightLoaderMotor.set(0);
    }
}
