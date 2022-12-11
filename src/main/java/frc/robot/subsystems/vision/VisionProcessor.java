package frc.robot.subsystems.vision;

import frc.robot.Constants.VisionProcessorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionProcessor extends SubsystemBase {

	// public static Drivetrain drivetrain;
	// private static Intake intake;

	private boolean led = false;
	// private boolean isAtY = false;
	public double robotSide;
	public boolean targetFound = false;
	// private boolean isCentered = false;
	// private boolean isAtArea = false;

	// Required Network Table Data
	private boolean seesTarget; // Get from Network Table
	private double tv;
	// private double yAngle;
	// private double difference;

	// private double xAngle; //Get from Network Table
	// private double area;
	// private double rotate = 0.0;
	// private double move = 0.0;

	// Accessing the Limelight's Network Table
	private NetworkTableInstance limeLightInstance = NetworkTableInstance.getDefault();
	private NetworkTable limeLightTable = limeLightInstance.getTable("/limelight");

	// Method for getting different data from a Network Table
	public double getNTInfo(String tableInfo) {
		NetworkTableEntry limeLightEntry = limeLightTable.getEntry(tableInfo);
		return limeLightEntry.getDouble(0);
	}

	// Method for setting different data into a Network Table
	public void setNTInfo(String tableInfo, int setValue) {
		NetworkTableEntry limeLightEntry = limeLightTable.getEntry(tableInfo);
		limeLightEntry.setNumber(setValue);
	}

	public VisionProcessor() {
		setName("Vision Processor");
	}

	public boolean seesTarget() {
		tv = getNTInfo("tv");
		if (tv != 0.0)
			seesTarget = true;
		else
			seesTarget = false;
		return seesTarget;
	}

	public void toggleLEDMode() {
		led = !led;
		if (led)
			setNTInfo("ledMode", VisionProcessorConstants.kVisionLedOn);
		else
			setNTInfo("ledMode", VisionProcessorConstants.kVisionLedOff);
	}

	public double getRotate() {
		// return getNTInfo("tx");
		return -getNTInfo("ty");
	}

	// public void findTarget() { //drivetrain vision processing
	// double angle = getRotate();
	// if (Math.abs(angle) > 1.0) {
	// Swerve.getInstance().rotateDegreesfromPosition(angle);
	// targetFound = false;
	// }
	// else
	// targetFound = true;
	// }

	public double getDistance() {
		if (seesTarget())
			// return 74/Math.tan(Math.PI*((getNTInfo("ty")+20)/180));
			return 72.5 / Math.tan(Math.PI * ((getNTInfo("tx") + 40) / 180)); // target 94" - camera height 21.5"
																				// ty = camera angle + Ty
		else
			return 0;
	}

	public void sendToDashboard() {
		SmartDashboard.putBoolean("Vision Has Target", targetFound);
		SmartDashboard.putNumber("Vision_Distance ", getDistance());
		SmartDashboard.putNumber("Vision tx", getRotate());
	}

}
