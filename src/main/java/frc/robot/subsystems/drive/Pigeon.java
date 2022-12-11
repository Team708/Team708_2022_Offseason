package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon {

	private static Pigeon instance = null;

	public static Pigeon getInstance() {
		if (instance == null) {
			instance = new Pigeon();
		}
		return instance;
	}

	private PigeonIMU pigeon;
	PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();

	private Pigeon() {
		try {
			pigeon = new PigeonIMU(0);
			pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 5, 10);
			pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 5, 10);

		} catch (Exception e) {
			System.out.println(e);
		}
	}

	public boolean isGood() {
		return (pigeon.getState() == PigeonState.Ready) ? true : false;
	}

	public Rotation2d getAngle() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return Rotation2d.fromDegrees(ypr[0]);
	}

	public Rotation2d getPitch() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return Rotation2d.fromDegrees(ypr[1]);
	}

	public Rotation2d getRoll() {
		double[] ypr = new double[3];
		pigeon.getYawPitchRoll(ypr);
		return Rotation2d.fromDegrees(ypr[2]);
	}

	public double getRateX() {
		double[] ypr = new double[3];
		pigeon.getRawGyro(ypr);
		return ypr[0];
	}

	public double getRateY() {
		double[] ypr = new double[3];
		pigeon.getRawGyro(ypr);
		return ypr[1];
	}

	public double getRateZ() {
		double[] ypr = new double[3];
		pigeon.getRawGyro(ypr);
		return ypr[2];
	}

	public double getRawAngle() {
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		return -pigeon.getFusedHeading(fusionStatus);
	}

	public void setAngle(double angle) {
		pigeon.setFusedHeading(-angle * 64, 10);
		pigeon.setYaw(-angle * 64, 10);
	}

	public void reset() {
		setAngle(0);
	}

	public void outputToSmartDashboard() {
		SmartDashboard.putBoolean("Pigeon Good", isGood());
		SmartDashboard.putNumber("Pigeon Temp", pigeon.getTemp());
		// SmartDashboard.putNumber("Pigeon Compass",
		// pigeon.getAbsoluteCompassHeading());
		SmartDashboard.putNumber("Pigeon Yaw", getAngle().getDegrees());
	}

}