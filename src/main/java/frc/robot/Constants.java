// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class RobotConstants {
    public static final boolean kRobot = true; // snowflake = true, competition = false
  }

  public static final class DriveConstants {

    public static final int kFrontLeftDriveMotorPort    = 12; // CAN ID
    public static final int kRearLeftDriveMotorPort     = 13; // CAN ID
    public static final int kFrontRightDriveMotorPort   = 11; // CAN ID
    public static final int kRearRightDriveMotorPort    = 14; // CAN ID

    public static final int kFrontLeftTurningMotorPort  = 16; // CAN ID
    public static final int kRearLeftTurningMotorPort   = 17; // CAN ID
    public static final int kFrontRightTurningMotorPort = 15; // CAN ID
    public static final int kRearRightTurningMotorPort  = 18; // CAN ID

    public static final boolean kFrontLeftDriveEncoderReversed  = (RobotConstants.kRobot) ? true : false;
    public static final boolean kRearLeftDriveEncoderReversed   = (RobotConstants.kRobot) ? true : false;
    public static final boolean kFrontRightDriveEncoderReversed = (RobotConstants.kRobot) ? false : false;
    public static final boolean kRearRightDriveEncoderReversed  = (RobotConstants.kRobot) ? false : false;

    // offset in degrees			
    public static final double kFrontLeftOffset   = (RobotConstants.kRobot) ? 86 : -34.2;    // degree
    public static final double kFrontRightOffset  = (RobotConstants.kRobot) ? 125.5 : -93.2; // degree
    public static final double kRearLeftOffset    = (RobotConstants.kRobot) ? 30.5 : -68.2;  // degree
    public static final double kRearRightOffset   = (RobotConstants.kRobot) ? 24.1 : -136.4; // degree

    // Distance between centers of right and left wheels on robot    
    public static final double kTrackWidth = (RobotConstants.kRobot) ? 0.6731 : 0.444; // m

    // Distance between front and back wheels on robot
    public static final double kWheelBase = (RobotConstants.kRobot) ? 0.8382 : 0.444; // m

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = true;

    public static final double kMaxSpeedMetersPerSecond = 25; // m/s
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 10 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 8 * Math.PI;

    public static final int kEncoderCPR = 42;
    public static final double kEncoderRatio = .119;
    public static final double kWheelDiameterMeters = 0.1;
    public static final double kDriveEncoderVelocityPerPulse = 0.00107063517;
    public static final double kVelocityModifier = (RobotConstants.kRobot) ? 0.8 : 0.85;
    public static final double kDriveEncoderDistancePerPulse = ((kEncoderCPR * kEncoderRatio) / Math.PI
        * (kWheelDiameterMeters) / 3.056814908981323);

    public static final double kTurningEncoderDistancePerPulse = (2 * Math.PI) / (double) 4096;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 0.1;
  }
  
  public static final class IntakeConstants  {
  // public static int kIntakeMotorPort          = 21; // CAN ID

  }

  public static final class ControllerConstants {

    public static final int kDriverControllerPort     = 0;
    public static final int kOperatorControllerPort   = 1;
    
    // Driver
    public static final double kDriverDeadBandLeftX   = 0.1;
    public static final double kDriverDeadBandRightX  = 0.2;
    public static final double kDriverDeadBandLeftY   = 0.1;
    public static final double kDriverDeadBandRightY  = 0.2;

    // Operator
    // public static final double kOperatorDeadBandLeftX   = 0.1;
    // public static final double kOperatorDeadBandRightX  = 0.2;
    // public static final double kOperatorDeadBandLeftY   = 0.1;
    // public static final double kOperatorDeadBandRightY  = 0.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 6; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = 1 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 1 * Math.PI;
    
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VisionProcessorConstants {

    public static final int CANdleID = 1;

    public static final int kVisionLedOn = 0;
    public static final int kVisionLedOff = 1;
  }
  
  public static final class FMSConstants {
    public static final int ALLIANCE_RED 	 		    = 1;
    public static final int ALLIANCE_BLUE 	 		  = -1;
    public static final int ALLIANCE_INITIALIZED  = 0;
    public static final int ALLIANCE_NOT_ENABLED  = 20;
    public static final int ALLIANCE_EXCEPTION    = 11;
  }
}
