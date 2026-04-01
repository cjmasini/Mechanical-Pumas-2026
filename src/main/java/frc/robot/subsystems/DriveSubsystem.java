package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.RobotConstants.Direction;
import frc.robot.utils.SwerveUtils;
import frc.robot.vision.LimelightHelpers;

public class DriveSubsystem extends CancelableSubsystemBase {

  // Create Swerve Modules
  private final SwerveModule frontLeftModule = new SwerveModule(
      CANIdConstants.FRONT_LEFT_DRIVE_CAN_ID,
      CANIdConstants.FRONT_LEFT_STEERING_CAN_ID,
      RobotConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule frontRightModule = new SwerveModule(
      CANIdConstants.FRONT_RIGHT_DRIVE_CAN_ID,
      CANIdConstants.FRONT_RIGHT_STEERING_CAN_ID,
      RobotConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule backLeftModule = new SwerveModule(
      CANIdConstants.BACK_LEFT_DRIVE_CAN_ID,
      CANIdConstants.BACK_LEFT_STEERING_CAN_ID,
      RobotConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule backRightModule = new SwerveModule(
      CANIdConstants.BACK_RIGHT_DRIVE_CAN_ID,
      CANIdConstants.BACK_RIGHT_STEERING_CAN_ID,
      RobotConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // The gyro sensor
  private final Pigeon2 gyro = new Pigeon2(CANIdConstants.PIGEON_GYRO_CAN_ID);

  // PID Controller for orientation to supplied angle
  private final PIDController orientationController;

  // Pose estimator for tracking robot pose with odometry + vision
  private final SwerveDrivePoseEstimator poseEstimator;

  public DriveSubsystem() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::updatePose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
          AutonConstants.AUTON_CONTROLLER, // PID controller for autonomous driving
          config, // RobotConfig object that holds your robot's physical properties
          () -> {
            return (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? true : false;
          }, // Supplier that returns true if the robot is on the red alliance
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    poseEstimator = new SwerveDrivePoseEstimator(
        RobotConstants.DRIVE_KINEMATICS,
        Rotation2d.fromDegrees(getGyroOrientation()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        },
        new Pose2d());

    orientationController = new PIDController(.01, 0, 0);
    orientationController.enableContinuousInput(-180, 180);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    poseEstimator.update(
        Rotation2d.fromDegrees(getGyroOrientation()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        });

    logPose("Odometry", getPose());
    SmartDashboard.putNumber("gyroOrientation", getGyroOrientation());
    SmartDashboard.putNumber("Debug/RawHeading", getHeading());
    SmartDashboard.putNumber("Debug/PoseHeading", getPose().getRotation().getDegrees());

    // Fault monitoring for FL and BR modules (the problematic ones)
    logModuleDiagnostics("FL", frontLeftModule);
    logModuleDiagnostics("BR", backRightModule);
  }

  /**
   * Logs diagnostic information for a swerve module to SmartDashboard.
   *
   * @param name   The name prefix for the module (e.g., "FL", "BR")
   * @param module The swerve module to log diagnostics for
   */
  private void logModuleDiagnostics(String name, SwerveModule module) {
    var faults = module.getDriveFaults();
    var warnings = module.getDriveWarnings();

    // Log faults and warnings as strings (contains all active fault names)
    SmartDashboard.putString(name + "_DriveFaults", faults.toString());
    SmartDashboard.putString(name + "_DriveWarnings", warnings.toString());
    SmartDashboard.putString(name + "_TurnFaults", module.getTurnFaults().toString());

    // Log motor outputs and encoder data
    SmartDashboard.putNumber(name + "_DriveAppliedOutput", module.getDriveAppliedOutput());
    SmartDashboard.putNumber(name + "_DriveCurrent", module.getDriveOutputCurrent());
    SmartDashboard.putNumber(name + "_TurnAngle", Math.toDegrees(module.getTurnEncoderPosition()));
    SmartDashboard.putNumber(name + "_DriveVelocity", module.getState().speedMetersPerSecond);
  }

  /**
   * Method to drive the robot while it adjusts to a specified orientation.
   *
   * @param xSpeed
   *                  Speed of the robot in the x direction (forward).
   * @param ySpeed
   *                  Speed of the robot in the y direction (sideways).
   * @param direction
   *                  Direction to orient front of robot towards.
   */
  public void driveAndOrient(double xSpeed, double ySpeed, Direction direction) {
    this.driveAndOrient(xSpeed, ySpeed,
        SwerveUtils.normalizeAngle(SwerveUtils.directionToAngle(direction, this.getHeading())));
  }

  /**
   * Method to drive the robot while it adjusts to a specified orientation.
   *
   * @param xSpeed
   *                      Speed of the robot in the x direction (forward).
   * @param ySpeed
   *                      Speed of the robot in the y direction (sideways).
   * @param targetHeading
   *                      Target heading (angle) robot should face
   */
  public void driveAndOrient(double xSpeed, double ySpeed, double target) {
    driveAndOrient(xSpeed, ySpeed, target, true);
  }

  /**
   * Drives the robot while automatically orienting towards a detected AprilTag.
   * If no tag is detected, manual rotation is used.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot    Manual angular rate (used if no target is found).
   */
  public void driveWhileTargeting(double xSpeed, double ySpeed, double rot) {
    boolean hasTarget = LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
    if (hasTarget) {
      double tx = LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
      double targetHeading = getHeading() - tx;
      driveAndOrient(xSpeed, ySpeed, targetHeading, true);
    } else {
      drive(xSpeed, ySpeed, rot, true);
    }
  }

  /**
   * Method to drive the robot while it adjusts to a specified orientation.
   *
   * @param xSpeed
   *                      Speed of the robot in the x direction (forward).
   * @param ySpeed
   *                      Speed of the robot in the y direction (sideways).
   * @param target
   *                      Target heading (angle) robot should face
   * @param fieldRelative
   *                      Whether the robot is in field relative mode
   */
  public void driveAndOrient(double xSpeed, double ySpeed, double target, boolean fieldRelative) {
    double currentHeading = this.getPose().getRotation().getDegrees();
    double targetHeading = SwerveUtils.normalizeAngle(target);

    // The left stick controls translation of the robot.
    // Automatically turn to face the supplied heading
    this.drive(
        xSpeed,
        ySpeed,
        this.orientationController.calculate(currentHeading, targetHeading),
        fieldRelative);
  }

  /**
   * Drive in a robot relative direction.
   * Accepts ChassisSpeeds object for path planner integration.
   * 
   * @param robotRelativeSpeeds
   */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = RobotConstants.DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed
   *                      Speed of the robot in the x direction (forward).
   * @param ySpeed
   *                      Speed of the robot in the y direction (sideways).
   * @param rot
   *                      Angular rate of the robot.
   * @param fieldRelative
   *                      Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
    double ySpeedDelivered = ySpeed * ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
    double rotDelivered = rot * ModuleConstants.MAX_ANGULAR_SPEED;

    var swerveModuleStates = RobotConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getGyroOrientation()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    backLeftModule.setDesiredState(swerveModuleStates[2]);
    backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose (x / y coordinates and rotation)
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Updates the odometry to the specified pose.
   *
   * @param pose
   *             The pose to which to set the odometry.
   */
  public void updatePose(Pose2d pose) {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(getGyroOrientation()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        },
        pose);
  }

  /**
   * Adds a vision-based pose measurement into the pose estimator.
   *
   * @param visionRobotPoseMeters    Robot pose from vision
   * @param timestampSeconds         Timestamp of the vision measurement (in
   *                                 seconds).
   * @param visionMeasurementStdDevs Std devs for [x, y, theta] in meters /
   *                                 radians.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters,
        timestampSeconds,
        visionMeasurementStdDevs);
  }

  /**
   * Gets current speeds of the robot in robot-relative coordinates.
   * 
   * @return The current ChassisSpeeds in robot-relative frame.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return RobotConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get current states of the swerve modules
   * 
   * @return The current SwerveModule states.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    };
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates
   *                      The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    backLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backRightModule.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return SwerveUtils.normalizeAngle(getGyroOrientation());
  }

  /**
   * Logs the components of a Pose2d to the SmartDashboard.
   * 
   * @param name The base name for the pose entries.
   * @param pose The Pose2d to log.
   */
  private void logPose(String name, Pose2d pose) {
    SmartDashboard.putNumber(name + "_x", pose.getX());
    SmartDashboard.putNumber(name + "_y", pose.getY());
    SmartDashboard.putNumber(name + "_rot", pose.getRotation().getDegrees());
  }

  /**
   * Builds a PathPlannerPath from the current robot pose to the supplied target
   * pose.
   * 
   * @param targetPose The desired target pose
   * @return The generated PathPlannerPath
   */
  public PathPlannerPath buildPathToPose(Pose2d targetPose) {
    Pose2d startPose = new Pose2d(
        getPose().getTranslation(),
        getPose().getRotation());

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, targetPose);

    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        AutonConstants.ON_THE_FLY_PATH_CONSTRAINTS,
        null,
        new GoalEndState(
            0.0,
            targetPose.getRotation()));

    path.preventFlipping = true;

    return path;
  }

  /**
   * Drives the robot to the supplied field-relative pose (For the blue side)
   * by generating an on-the-fly PathPlanner path from the current pose to
   * the target and scheduling it through AutoBuilder.
   *
   * @param destinationPose Desired Pose2d on the field.
   */
  public void driveToPose(Pose2d destinationPose) {
    PathPlannerPath path = buildPathToPose(destinationPose);

    AutoBuilder
        .followPath(path)
        .withTimeout(3.0)
        .schedule();
  }

  /**
   * Creates a command that drives the robot to the supplied field-relative pose
   * 
   * @param targetPose The desired target pose
   * @return The command to drive to the pose
   */
  public Command createDriveToPoseCommand(Pose2d targetPose) {
    PathPlannerPath path = buildPathToPose(targetPose);
    return AutoBuilder.followPath(path).withTimeout(3);
  }

  /**
   * Sets the brake mode for all four swerve modules.
   * 
   * @param brake If true, sets motors to brake mode; if false, sets to coast
   *              mode.
   */
  public void setModuleBrakeModes(boolean brake) {
    frontLeftModule.setBrakeMode(brake);
    frontRightModule.setBrakeMode(brake);
    backLeftModule.setBrakeMode(brake);
    backRightModule.setBrakeMode(brake);
  }

  /**
   * Gets the yaw orientation from the gyro.
   * 
   * @return The gyro yaw orientation in degrees.
   */
  public double getGyroOrientation() {
    return gyro.getYaw().getValueAsDouble() + RobotConstants.GYRO_OFFSET;
  }

  /**
   * Sets robot speed to zero.
   */
  @Override
  public void cancel() {
      drive(0, 0, 0, true);
  }
}
