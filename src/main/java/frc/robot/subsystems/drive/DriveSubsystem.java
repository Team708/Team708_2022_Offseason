// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule("frontLeft", DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftOffset);

  private final SwerveModule m_rearLeft = new SwerveModule("rearLeft", DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort, DriveConstants.kRearLeftDriveEncoderReversed,
      DriveConstants.kRearLeftOffset);

  private final SwerveModule m_frontRight = new SwerveModule("frontRight", DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightOffset);

  private final SwerveModule m_rearRight = new SwerveModule("rearRight", DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort, DriveConstants.kRearRightDriveEncoderReversed,
      DriveConstants.kRearRightOffset);

  // The gyro sensor
  private final Pigeon m_gyro = Pigeon.getInstance();

  public static int speedCoeff = 10;
  public int speedLevel = 4;

  private PIDController turnPID = new PIDController(15, 0, 0);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getHeading()));

  public DifferentialDrivetrainSim m_drivetrainSimulator;


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    turnPID.enableContinuousInput(-Math.PI, Math.PI);
    turnPID.setTolerance(0.02, 0.5);
  }

  public int getSpeedCoeff() {
    return speedCoeff;
  }

  public void increaseSpeed() {
    switch (speedLevel) {
      case 0:
        speedCoeff = 3;
        break;
      case 1:
        speedCoeff = 6;
        break;
      case 2:
        speedCoeff = 8;
        break;
      case 3:
        speedCoeff = 10;
        break;
      case 4:
        speedCoeff = 1;
        break;
    }
    speedLevel = (speedLevel + 1) % 5;
  }

  public void decreaseSpeed() {
    switch (speedLevel) {
      case 0:
        speedCoeff = 10;
        break;
      case 1:
        speedCoeff = 1;
        break;
      case 2:
        speedCoeff = 3;
        break;
      case 3:
        speedCoeff = 6;
        break;
      case 4:
        speedCoeff = 8;
        break;
    }
    speedLevel = Math.floorMod((speedLevel - 1), 5);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_rearLeft.getState(), m_frontLeft.getState(),
        m_rearRight.getState(), m_frontRight.getState());
  }

  @Override
  public void simulationPeriodic() {

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
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
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
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // if (rot == 0 && (xSpeed != 0 || ySpeed !=0)) {
    //   rot = turnPID.calculate(m_gyro.getAngle().getRadians());
    // } else {
    //   turnPID.setSetpoint(m_gyro.getAngle().getRadians());
    // }

    SmartDashboard.putNumber("rot", rot);
    SmartDashboard.putNumber("setpoint", turnPID.getSetpoint());

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
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
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return (Math.floorMod((long) m_gyro.getAngle().getDegrees(), (long) 360));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRateX();
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void invertDrive(){
    m_frontLeft.invertDrive();
    m_frontRight.invertDrive();
    m_rearLeft.invertDrive();
    m_rearRight.invertDrive();
  }

  public void sendToDashboard() {
    // SmartDashboard.putNumber("State rad FL",
    // m_frontLeft.getState().angle.getDegrees() % 360);
    // SmartDashboard.putNumber("State rad RL",
    // m_rearLeft.getState().angle.getDegrees() % 360);
    // SmartDashboard.putNumber("State rad FR",
    // m_frontRight.getState().angle.getDegrees() % 360);
    // SmartDashboard.putNumber("State rad RR",
    // m_rearRight.getState().angle.getDegrees() % 360);
    // SmartDashboard.putNumber("State m/s FL",
    // m_frontLeft.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("State m/s RL:",
    // m_rearLeft.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("State m/s FR",
    // m_frontRight.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("State m/s RR",
    // m_rearRight.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("turn rate", getTurnRate());
    SmartDashboard.putNumber("heading", getHeading());
    SmartDashboard.putNumber("Drive Coeff", speedCoeff);
    SmartDashboard.putNumber("pose x", getPose().getTranslation().getX());
    SmartDashboard.putNumber("pose y", getPose().getTranslation().getY());
    SmartDashboard.putNumber("pose hyp", getPose().getTranslation().getNorm());
    SmartDashboard.putNumber("pose rot", getPose().getRotation().getDegrees());
    m_gyro.outputToSmartDashboard();
    m_frontLeft.sendToDashboard();
    m_rearLeft.sendToDashboard();
    m_frontRight.sendToDashboard();
    m_rearRight.sendToDashboard();

  }
}
