package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final TalonSRX m_turningMotor;
  private final String modID;
  private final double offset;
  private final RelativeEncoder m_driveEncoder;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController, 0, 0,
      new TrapezoidProfile.Constraints(ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(String modID, int driveMotorChannel, int turningMotorChannel, boolean driveEncoderReversed,
      double offset) {
    this.modID = modID;
    this.offset = offset;
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new TalonSRX(turningMotorChannel);

    m_driveEncoder = m_driveMotor.getEncoder();

    configureMotors();
    resetEncoders();

    // Set whether drive encoder should be reversed or not
    m_driveMotor.setInverted(driveEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private void configureMotors() {
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    m_turningMotor.setSensorPhase(true);
    m_turningMotor.setInverted(true);
    m_turningMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
    m_turningMotor.enableVoltageCompensation(true);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.configVoltageCompSaturation(7.0, 10);
    m_turningMotor.configNominalOutputForward(0.0, 10);
    m_turningMotor.configNominalOutputReverse(0.0, 10);
    m_turningMotor.configAllowableClosedloopError(0, 0, 10);
    m_turningMotor.configMotionAcceleration((int) (1992 * 1.0), 10); // 10.0 jnp
    m_turningMotor.configMotionCruiseVelocity((int) (1992 * 1.0), 10);// 0.8 jnp
    m_turningMotor.selectProfileSlot(0, 0);
    m_turningMotor.config_kP(0, 4.0, 10);// 1
    m_turningMotor.config_kI(0, 0.0, 10);
    m_turningMotor.config_kD(0, 80.0, 10);// 10
    m_turningMotor.config_kF(0, 0.75 * (1023.0 / 1992), 10);
    m_turningMotor.set(ControlMode.MotionMagic, m_turningMotor.getSelectedSensorPosition(0));

    // drivePIDController.setFeedbackDevice(driveEncoder);
    m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderDistancePerPulse);
    m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityPerPulse * ModuleConstants.kVelocityModifier);
    m_driveEncoder.setPosition(0.0);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    // drivePIDController.setP(0.2);
    // drivePIDController.setI(0);
    // drivePIDController.setD(24);
  }

  /**
   * Returns the current angle the module.
   *
   * @return The current angle of the module in radians.
   */
  public double getAngle() {
    return (m_turningMotor.getSelectedSensorPosition(0) * ModuleConstants.kTurningEncoderDistancePerPulse)
        + (offset * Math.PI / 180);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_turningPIDController.calculate(getAngle(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    SmartDashboard.putNumber(modID + "driveOutput", driveOutput);
    m_driveMotor.set(driveOutput);
    SmartDashboard.putNumber(modID + "turnOutput", turnOutput);
    m_turningMotor.set(TalonSRXControlMode.PercentOutput, turnOutput);
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    // m_turningMotor.setSelectedSensorPosition(0);
  }

  public void invertDrive(){
    if (m_driveMotor.getInverted()){
      m_driveMotor.setInverted(false);
    }
    else {
      m_driveMotor.setInverted(true);
    }
  }

  public void sendToDashboard() {
    // SmartDashboard.putNumber(modID + " drive pos", m_driveEncoder.getPosition());
    SmartDashboard.putNumber(modID + " drive vel", m_driveEncoder.getVelocity());
    SmartDashboard.putNumber(modID + " turn pos",
    ((180/Math.PI)*(m_turningMotor.getSelectedSensorPosition(0) * ModuleConstants.kTurningEncoderDistancePerPulse)
    + (offset * Math.PI / 180)));
    // SmartDashboard.putNumber(modID + "state velocity", getState().speedMetersPerSecond);
    // SmartDashboard.putNumber(modID + " turn vel",
    // m_turningMotor.getSelectedSensorVelocity(0) *
    // ModuleConstants.kTurningEncoderDistancePerPulse *180/Math.PI);
  }
}
