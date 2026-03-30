package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
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
    private double maxDistance = -Double.MAX_VALUE;

    private final double LAUNCHER_VELOCITY_TOLERANCE = 20;

    private double targetVelocity = 0.0;

    /**
     * Initialize the Launch Subsystem.
     */
    public LaunchSubsystem() {
        this.setName("LaunchSubsystem");
        prepareLaunchMotors();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("launch/leftVelocity", leftLauncherEncoder.getVelocity());
                SmartDashboard.putNumber("launch/rightVelocity", rightLauncherEncoder.getVelocity());

    }

    private void prepareLaunchMotors() {
         this.leftLauncherMotor = new SparkFlex(
                CANIdConstants.LEFT_LAUNCHER_ID,
                MotorType.kBrushless);

        this.rightLauncherMotor = new SparkFlex(
                CANIdConstants.RIGHT_LAUNCHER_ID,
                MotorType.kBrushless);

        SparkFlexConfig launcherConfig = new SparkFlexConfig();
    
        launcherConfig.idleMode(IdleMode.kBrake);

        launcherConfig.smartCurrentLimit(40);
        launcherConfig.voltageCompensation(12.0);
    
        launcherConfig.closedLoop
            .pid(0.0005, 0.0, 0.00005 )
            .outputRange(-1.0, 1.0)
            .feedForward.kV(0.0018);
        launcherConfig.inverted(false);

        leftLauncherMotor.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);



        rightLauncherMotor.configure(
                launcherConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    
    
        this.leftClosedLoopController = leftLauncherMotor.getClosedLoopController();
        this.leftLauncherEncoder = leftLauncherMotor.getEncoder();
        this.rightClosedLoopController = rightLauncherMotor.getClosedLoopController();
        this.rightLauncherEncoder = rightLauncherMotor.getEncoder();

        createDistanceMap();
    }


    private void createDistanceMap() {
        addCalibrationPoint(Units.inchesToMeters(41), 2900);
        addCalibrationPoint(Units.inchesToMeters(53), 2950);
        addCalibrationPoint(Units.inchesToMeters(66), 3000);
        addCalibrationPoint(Units.inchesToMeters(67.5), 3050);
        addCalibrationPoint(Units.inchesToMeters(68), 3100);
        addCalibrationPoint(Units.inchesToMeters(74), 3200);
        addCalibrationPoint(Units.inchesToMeters(80), 3250);
        addCalibrationPoint(Units.inchesToMeters(83), 3300);
        addCalibrationPoint(Units.inchesToMeters(90), 3350);
        addCalibrationPoint(Units.inchesToMeters(95), 3400);
        addCalibrationPoint(Units.inchesToMeters(99), 3450);
        addCalibrationPoint(Units.inchesToMeters(103), 3500);
        addCalibrationPoint(Units.inchesToMeters(106), 3550);
        addCalibrationPoint(Units.inchesToMeters(112), 3600);
        addCalibrationPoint(Units.inchesToMeters(119), 3650);

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
     * @param power Speed (-1 to 1) to set the launcher motor at.
     */
    public void setLauncherPower(double power) {
        this.leftLauncherMotor.set(power);
        this.rightLauncherMotor.set(-power);
    }
    /**
     * Set the launcher motor to a target velocity
     *
     * @param rpm Target velocity in RPM
     */
    public void setLauncherVelocity(double rpm) {
        this.targetVelocity = rpm;
        SmartDashboard.putNumber("launch/commandedRPM", rpm);
        leftClosedLoopController.setSetpoint(rpm, ControlType.kVelocity);
        rightClosedLoopController.setSetpoint(-rpm, ControlType.kVelocity);
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
        return (Math.abs(leftLauncherEncoder.getVelocity()) + Math.abs(rightLauncherEncoder.getVelocity())) / 2.0;
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
    public boolean isAtTargetVelocity() {
        return Math.abs(getLauncherVelocity() - targetVelocity) < LAUNCHER_VELOCITY_TOLERANCE;
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
