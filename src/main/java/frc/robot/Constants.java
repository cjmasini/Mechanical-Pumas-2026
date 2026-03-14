package frc.robot;

import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class CANIdConstants {
        // Drive Motor CAN Ids
        public static final int FRONT_RIGHT_DRIVE_CAN_ID = 1;
        public static final int BACK_RIGHT_DRIVE_CAN_ID = 3;
        public static final int BACK_LEFT_DRIVE_CAN_ID = 5;
        public static final int FRONT_LEFT_DRIVE_CAN_ID = 7;

        public static final int FRONT_RIGHT_STEERING_CAN_ID = 2;
        public static final int BACK_RIGHT_STEERING_CAN_ID = 4;
        public static final int BACK_LEFT_STEERING_CAN_ID = 6;
        public static final int FRONT_LEFT_STEERING_CAN_ID = 8;

        // Example Motor CAN Ids
        public static final int LEFT_LAUNCHER_ID = 41;
        public static final int RIGHT_LAUNCHER_ID = 42;

        public static final int LEFT_LOADER_ID = 43;
        public static final int RIGHT_LOADER_ID = 44;
        public static final int INDEXER_MOTOR_ID = 45;

        public static final int LEADER_DEPLOY_ID = 51;
        public static final int FOLLOWER_DEPLOY_ID = 52;
        public static final int ROLLER_ID = 53;


        public static final int PIGEON_GYRO_CAN_ID = 21;
    }

    public static final class RobotConstants {
        // Chassis configuration
        public static final double WHEEL_BASE_WIDTH = Units.inchesToMeters(30);
        public static final double WHEEL_BASE_LENGTH = Units.inchesToMeters(24.5);

        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        // Distance between centers of right and left or front and back wheels on robot
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE_WIDTH / 2, WHEEL_BASE_LENGTH / 2),
                new Translation2d(WHEEL_BASE_WIDTH / 2, -WHEEL_BASE_LENGTH / 2),
                new Translation2d(-WHEEL_BASE_WIDTH / 2, WHEEL_BASE_LENGTH / 2),
                new Translation2d(-WHEEL_BASE_WIDTH / 2, -WHEEL_BASE_LENGTH / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI/2;
        public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
        public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
        public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        public static final double GYRO_OFFSET = 180;
        public static final boolean kGyroReversed = false;

        // Enum for auto-orienting to field directions
        public enum Direction {
            FORWARD, BACKWARD, LEFT, RIGHT
        }
    }

    public static final class ModuleConstants {
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

        // Calculations required for driving motor conversion factors and feed forward

        // The MAXSwerve module can be configured with one of three pinion gears: 12T
        // (Low),
        // 13T (Medium), or 14T (High).

        public static final int DRIVE_MOTOR_PINION_TEETH = 13;

        public static final double DRIVE_MOTOR_FREE_SPEED_RPM = 6784;
        public static final double WHEEL_CIRCUMFERENCE_IN_METERS = 0.23938936;
        public static final double WHEEL_RADIUS_IN_METERS = 0.0762;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_IN_MPS = (DRIVE_MOTOR_FREE_SPEED_RPM / 60
                * WHEEL_CIRCUMFERENCE_IN_METERS)
                / DRIVE_MOTOR_REDUCTION;
    }

    public static final class OperatorConstants {
        public static final double DRIVE_DEADBAND = 0.05;
        public static final double DPAD_SPEED_REGULATOR = .25;
    }

    public static final class AutonConstants {
        // PID constants for holonomic auto control (PathPlanner)
        // TODO: Tune these values on the real robot
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.3, 0.0, 0.0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.01, 0.0, 0.0);

        // Holonomic controller used by AutoBuilder
        public static final PathFollowingController AUTON_CONTROLLER = new PPHolonomicDriveController(
                TRANSLATION_PID,
                ANGLE_PID);

        // TODO: Fine tune these values
        // On-the-fly PathPlanner constraints
        // Linear constraints (m/s, m/s^2)
        public static final double MAX_AUTO_SPEED_MPS = 3.0;
        public static final double MAX_AUTO_ACCEL_MPS2 = 3.0;

        // Angular constraints (rad/s, rad/s^2)
        public static final double MAX_AUTO_ANGULAR_SPEED_RAD_PER_SEC = Math.toRadians(360.0);
        public static final double MAX_AUTO_ANGULAR_ACCEL_RAD_PER_SEC = Math.toRadians(540.0);

        public static final PathConstraints ON_THE_FLY_PATH_CONSTRAINTS = new PathConstraints(
                AutonConstants.MAX_AUTO_SPEED_MPS,
                AutonConstants.MAX_AUTO_ACCEL_MPS2,
                AutonConstants.MAX_AUTO_ANGULAR_SPEED_RAD_PER_SEC,
                AutonConstants.MAX_AUTO_ANGULAR_ACCEL_RAD_PER_SEC);
    }

    public static final class VisionConstants {
        public static final AprilTagFieldLayout APRIL_TAG_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        public static final String FRONT_RIGHT_CAMERA_NAME = "limelight";

        // +X = forward, +Y = left, +Z = up (meters from robot center)
        private static final Translation3d FRONT_RIGHT_CAMERA_TRANSLATION = new Translation3d(
                Units.inchesToMeters(-10.569981),   // X: forward
                Units.inchesToMeters(-11.904615),   // Y: left
                Units.inchesToMeters(-7.703044)  // Z: height
        );
        // Camera orientation relative to robot frame (roll, pitch, yaw in degrees)
        private static final Rotation3d FRONT_RIGHT_LIMELIGHT_ROTATION = new Rotation3d(
                Math.toRadians(0.0),   // roll
                Math.toRadians(20.0), // pitch (looking slightly up)
                Math.toRadians(-15)    // yaw (facing slightly right)
        );
        public static final Transform3d FRONT_RIGHT_LIMELIGHT_TRANSFORM = new Transform3d(
                FRONT_RIGHT_CAMERA_TRANSLATION,
                FRONT_RIGHT_LIMELIGHT_ROTATION);
    }

    public static final class RebuiltConstants {
        // IDs for tags in the middle of the hub faces
        public static final Set<Integer> BLUE_CENTER_TAGS = Set.of(21, 26, 18);
        public static final Set<Integer> RED_CENTER_TAGS = Set.of(5, 10, 2);
        public static final Set<Integer> CENTER_TAGS = Stream.concat(BLUE_CENTER_TAGS.stream(), RED_CENTER_TAGS.stream()).collect(Collectors.toSet());
        // IDs for the off-center tags on the sides of the hub faces
        public static final Set<Integer> BLUE_OFFSET_TAGS = Set.of(24, 25, 27);
        public static final Set<Integer> RED_OFFSET_TAGS = Set.of(8, 10, 11);
        public static final Set<Integer> OFFSET_TAGS = Stream.concat(BLUE_OFFSET_TAGS.stream(), RED_OFFSET_TAGS.stream()).collect(Collectors.toSet());
     
        // Taken from pathplanner GUI
        public static final Translation2d RED_HUB_CENTER = new Translation2d(11.910, 4.030);
        public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.630, 4.030);
    }

}
