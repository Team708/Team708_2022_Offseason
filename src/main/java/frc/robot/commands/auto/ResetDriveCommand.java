package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetDriveCommand extends CommandBase {

    private final DriveSubsystem m_DriveSubsystem;
    private final Trajectory m_trajectory;

    public ResetDriveCommand(Trajectory trajectory, DriveSubsystem subsystem) {
        m_DriveSubsystem = subsystem;
        m_trajectory = trajectory;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.resetOdometry(m_trajectory.getInitialPose());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}