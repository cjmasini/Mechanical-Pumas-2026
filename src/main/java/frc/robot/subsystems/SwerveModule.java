// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;


import frc.robot.Constants.ModuleConstants;

/**
 * SwerveModule class - Tracks and controls state information for a swerve
 * module
 */
public class SwerveModule {
  private final SparkFlex driveMotorController;
  private final SparkMax turnMotorController;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  private final SparkClosedLoopController drivePIDController;
  private final SparkClosedLoopController turnPIDController;

  private double chassisAngularOffset = 0;

  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Configures a SwerveModule with a spark max motor controller and connected
   * through bore
   * encoder
   */
  public SwerveModule(int drivingCANId, int steeringCANId, double chassisAngularOffset) {
    this.driveMotorController = new SparkFlex(drivingCANId, MotorType.kBrushless);
    this.turnMotorController = new SparkMax(steeringCANId, MotorType.kBrushless);

    this.driveEncoder = driveMotorController.getEncoder();
    this.turnEncoder = turnMotorController.getAbsoluteEncoder();

    this.drivePIDController = driveMotorController.getClosedLoopController();
    this.turnPIDController = turnMotorController.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    this.driveMotorController.configure(SwerveModuleConfigs.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    this.turnMotorController.configure(SwerveModuleConfigs.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.chassisAngularOffset = chassisAngularOffset;
    this.desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(this.driveEncoder.getVelocity(),
        new Rotation2d(this.turnEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        this.driveEncoder.getPosition(),
        new Rotation2d(this.turnEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState
   *                     Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState(desiredState.speedMetersPerSecond,
        desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset)));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(this.turnEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    this.drivePIDController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    this.turnPIDController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    this.driveEncoder.setPosition(0);
  }

  /**
   * Sets the brake mode for both motors in the swerve module.
   * 
   * @param brake If true, sets motors to brake mode; if false, sets to coast
   *              mode.
   */
  public void setBrakeMode(boolean brake) {
    IdleMode mode = brake ? IdleMode.kBrake : IdleMode.kCoast;

    SwerveModuleConfigs.drivingConfig.idleMode(mode);
    SwerveModuleConfigs.turningConfig.idleMode(mode);

    // Reconfigure both motors without resetting other parameters
    this.driveMotorController.configure(
        SwerveModuleConfigs.drivingConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    this.turnMotorController.configure(
        SwerveModuleConfigs.turningConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  /**
   * Spark Max Configs for MaxSwerve Modules
   */
  private static final class SwerveModuleConfigs {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.WHEEL_CIRCUMFERENCE_IN_METERS
          / ModuleConstants.DRIVE_MOTOR_REDUCTION;
      double turningFactor = ModuleConstants.MAX_ANGULAR_SPEED;
      double drivingVelocityFeedForward = 1 / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .outputRange(-1, 1)
          .feedForward.kV(drivingVelocityFeedForward);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20);
      turningConfig.absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }
}