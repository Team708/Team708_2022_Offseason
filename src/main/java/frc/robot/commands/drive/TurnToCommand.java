package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** A command that will turn the robot to the specified angle. */
public class TurnToCommand extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public TurnToCommand(double targetAngleDegrees, DriveSubsystem drive) {
    super(
        new PIDController(0.2, 0, 0),
        drive::getHeading,
        targetAngleDegrees,
        output -> drive.drive(0, 0, -2 * output, true),
        drive);

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(1.5, 10);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
