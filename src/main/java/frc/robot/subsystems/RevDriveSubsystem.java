// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.utils.SwerveUtils;

public class RevDriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
    CANIdConstants.FRONT_LEFT_DRIVE_CAN_ID,
    CANIdConstants.FRONT_LEFT_STEERING_CAN_ID,
    RobotConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule m_frontRight = new SwerveModule(
      CANIdConstants.FRONT_RIGHT_DRIVE_CAN_ID,
      CANIdConstants.FRONT_RIGHT_STEERING_CAN_ID,
      RobotConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule m_rearLeft = new SwerveModule(
      CANIdConstants.BACK_LEFT_DRIVE_CAN_ID,
      CANIdConstants.BACK_LEFT_STEERING_CAN_ID,
      RobotConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule m_rearRight = new SwerveModule(
      CANIdConstants.BACK_RIGHT_DRIVE_CAN_ID,
      CANIdConstants.BACK_RIGHT_STEERING_CAN_ID,
      RobotConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // The gyro sensor
  private final Pigeon2 gyro = new Pigeon2(CANIdConstants.PIGEON_GYRO_CAN_ID);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      RobotConstants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(getGyroOrientation()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            });
      
        /** Creates a new DriveSubsystem. */
        public RevDriveSubsystem() {
          // Usage reporting for MAXSwerve template
          HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
        }
      
        private double getGyroOrientation() {
            return gyro.getYaw().getValueAsDouble() + RobotConstants.GYRO_OFFSET;
        }
      
      @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getGyroOrientation()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
     // m_frontLeft.resetMotorControllers();
     // m_frontRight.resetMotorControllers();
     // m_rearLeft.resetMotorControllers();
     // m_rearRight.resetMotorControllers();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getGyroOrientation()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * RobotConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * RobotConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * RobotConstants.kMaxAngularSpeed;

    var swerveModuleStates = RobotConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getGyroOrientation()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, RobotConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, RobotConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
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
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble() * (RobotConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
