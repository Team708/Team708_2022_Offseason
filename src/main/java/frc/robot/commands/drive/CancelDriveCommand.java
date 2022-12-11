package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class CancelDriveCommand extends CommandBase {

    private final DriveSubsystem m_DriveSubsystem;

    public CancelDriveCommand(DriveSubsystem subsystem) {
        m_DriveSubsystem = subsystem;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}