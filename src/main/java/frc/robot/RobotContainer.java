// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FMSConstants;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionProcessor;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final VisionProcessor m_visionProcessor = new VisionProcessor();
  public static final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Alliance Information
  public static DriverStation.Alliance alliance;
  public static int allianceColor;
  private NetworkTableEntry allianceColorEntry;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    CameraServer.startAutomaticCapture();

    OI.configureButtonBindings(m_robotDrive);

    // Configure default commands
    m_robotDrive.setDefaultCommand(new RunCommand(

      () -> m_robotDrive.drive(-m_robotDrive.getSpeedCoeff() * OI.getDriverLeftY(),
       -m_robotDrive.getSpeedCoeff() * OI.getDriverLeftX(),
       -m_robotDrive.getSpeedCoeff() * 25 / 10 * OI.getDriverRightX(), true),
      m_robotDrive));
      
    // Put the chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // this is where we run the initialization -see instantcommand: executes a
    // single action on initialization, and then ends immediately (on page
    // convenience features)
    return m_chooser.getSelected();
  }

  /**
   * Finds the trajectory on the rio
   *
   * @param json the mane of the json
   * @return the trajectory
   * 
   */
  public Trajectory findTrajectory(String json) {

    String trajectoryJSON = "paths/" + json + ".wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    return trajectory;
  }

  public Trajectory createTrejectory(Pose2d start, List<Translation2d> waypoints, Pose2d stop) {

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    return TrajectoryGenerator.generateTrajectory(start, waypoints, stop, config);
  }

  // findColor - sets the global variable allianceColor based on the color of the
  // alliance coming from the fmsdata
  public void findAllianceColor() {
    try {
      // robot is ENABLED
      if (RobotController.isSysActive()) {
        // connected to FMS
        allianceColor = FMSConstants.ALLIANCE_INITIALIZED; // 0
        alliance = DriverStation.getAlliance();

        String gameData = DriverStation.getGameSpecificMessage();
        SmartDashboard.putString("gameData", gameData);

        if (alliance == Alliance.Blue) {
          allianceColor = FMSConstants.ALLIANCE_BLUE; // -1
        } else if (alliance == Alliance.Red) {
          allianceColor = FMSConstants.ALLIANCE_RED; // 1
        } else {
          allianceColor = FMSConstants.ALLIANCE_INITIALIZED; // 0
        }
      } else { // robot is NOT ENABLED
        allianceColor = FMSConstants.ALLIANCE_NOT_ENABLED; // 20
      }

    } catch (Exception e) {
      allianceColor = FMSConstants.ALLIANCE_EXCEPTION; // 11
    }
    // example on updating the dashboard using the smart dashboard implementation
    SmartDashboard.putNumber("Alliance", allianceColor);

    // exmaple on updating the dashboard using the Shuffleboard custom widget and
    // the network table entry
    // allianceColorEntry.setDouble(allianceColor);

  }

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  public void  simulationPeriodic(){}

  public void sendToDashboard() {
    m_robotDrive.sendToDashboard();
    // m_shooter.sendToDashboard();
    // m_climber.sendToDashboard();
    // m_intakeFeeder.sendToDashboard();
    // m_limelight.sendToDashboard();
    // m_candleSystem.sendToDashboard();

  }

}
