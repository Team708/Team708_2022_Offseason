package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class IncreaseSpeedCommand extends CommandBase {

    private final DriveSubsystem m_DriveSubsystem;

    public IncreaseSpeedCommand(DriveSubsystem subsystem) {
        m_DriveSubsystem = subsystem;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.increaseSpeed();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
