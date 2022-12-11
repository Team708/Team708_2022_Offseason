package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SwerveCommand extends SequentialCommandGroup {

    public DriveSubsystem m_DriveSubsystem;

    public SwerveCommand(DriveSubsystem dSubsystem, Trajectory trajectory) {
       
        var thetaController = new ProfiledPIDController(0.5, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, dSubsystem::getPose,
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(10, 0, 0), new PIDController(10, 0, 0), thetaController,
                dSubsystem::setModuleStates, dSubsystem);

        addCommands(
                new ResetDriveCommand(trajectory, dSubsystem),
                swerveControllerCommand.andThen(() -> dSubsystem.drive(0, 0, 0, false)));

    }

}
