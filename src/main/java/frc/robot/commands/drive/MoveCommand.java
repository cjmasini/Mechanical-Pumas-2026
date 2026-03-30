package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command for moving the robot using the swerve drive modules and an x-box
 * controller
 */
public class MoveCommand extends Command {
  private final CommandXboxController driverXbox;
  private final DriveSubsystem driveSubsystem;
  private boolean fieldRelative = true;

  /**
   * Command for moving the robot using the swerve drive modules and an x-box
   * controller
   * D-PAD controls the robot at 20% speed in robot-oriented mode
   * Left Joystick controls the robot at variable speeds (0-100%) in
   * field-oriented mode
   * - Cubes driver input for greater control at low speeds
   * Right Joystick controls the robot orientation / heading (direction it is
   * facing)
   * Right Trigger + A/B/X/Y -> Robot automically orients to the corresponding
   * field-oriented heading
   * - Works while moving and continues to orient until button is released
   * 
   * @param drivetrain
   *                   The drive subsystem.
   * @param driverXbox
   *                   The xbox controller for the robot
   */
  public MoveCommand(DriveSubsystem drivetrain, CommandXboxController driverXbox) {
    this.driveSubsystem = drivetrain;
    this.driverXbox = driverXbox;
    addRequirements(this.driveSubsystem);
  }

  @Override
  public void execute() {
    // Trigger mappings for fine-tuned robot-oriented adjustments using the d-pad
    if (driverXbox.povLeft().getAsBoolean()) {
      driveSubsystem.drive(0, .2, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND),
          false);
    } else if (driverXbox.povRight().getAsBoolean()) {
      driveSubsystem.drive(0, -.2, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND),
          false);
    } else if (driverXbox.povUp().getAsBoolean()) {
      driveSubsystem.drive(.2, 0, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND),
          false);
    } else if (driverXbox.povDown().getAsBoolean()) {
      driveSubsystem.drive(-0.2, 0, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND),
          false);
    } else if (driverXbox.povUpLeft().getAsBoolean()) {
      driveSubsystem.drive(.14, .14, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND),
          false);
    } else if (driverXbox.povUpRight().getAsBoolean()) {
      driveSubsystem.drive(.14, -.14, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND),
          false);
    } else if (driverXbox.povDownLeft().getAsBoolean()) {
      driveSubsystem.drive(-.14, .14, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND),
          false);
    } else if (driverXbox.povDownRight().getAsBoolean()) {
      driveSubsystem.drive(-.14, -.14,
          -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND), false);
    } else { // Joystick / field-oriented based movement
      double yMovement = driverXbox.getLeftY();
      double xMovement = driverXbox.getLeftX();

      // Precompute processed joystick values (cubic scaling + deadband)
      double processedY = -MathUtil.applyDeadband(Math.pow(yMovement, 3), OperatorConstants.DRIVE_DEADBAND);
      double processedX = -MathUtil.applyDeadband(Math.pow(xMovement, 3), OperatorConstants.DRIVE_DEADBAND);
      double processedRot = -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND);

      // If cancel button is pressed, we may be in auto-orient mode
      // boolean orient = driverXbox.rightTrigger().getAsBoolean();
      Direction targetDirection = null;
      boolean orient = false;
      // // Choose direction based on face buttons
      // if (orient) {
      //   if (driverXbox.x().getAsBoolean()) {
      //     targetDirection = Direction.RIGHT;
      //   } else if (driverXbox.y().getAsBoolean()) {
      //     targetDirection = Direction.BACKWARD;
      //   } else if (driverXbox.b().getAsBoolean()) {
      //     targetDirection = Direction.LEFT;
      //   } else if (driverXbox.a().getAsBoolean()) {
      //     targetDirection = Direction.FORWARD;
      //   }
      // }

      // If auto-orient is active (cancel button held) and a direction button is pressed
      // if (orient && targetDirection != null) {
      //   this.driveSubsystem.driveAndOrient(processedY, processedX, targetDirection);

      // } else {
        // Default joystick-controlled swerve
        this.driveSubsystem.drive(
            processedY,
            processedX,
            processedRot,
            fieldRelative);
      // }

    }
  }

  /*
   * toggleFieldRelative - toggles the drive mode of the robot
   * 
   * Can be used to dynamically switch between robot-oriented and field-oriented
   * modes
   */
  public void toggleFieldRelative() {
    this.fieldRelative = !this.fieldRelative;
  }
}
