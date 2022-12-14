package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.drive.ResetGyroCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drive.DriveSubsystem;

public class OI {

	// Gamepads
	public final static XboxController driverGamepad = new XboxController(ControllerConstants.kDriverControllerPort); // Driver
	public final static XboxController operatorGamepad = new XboxController(ControllerConstants.kOperatorControllerPort); // Operator
	// public final static XboxController climberGamepad  = new XboxController(ControllerConstants.kClimberControllerPort); // Climber
	// public final static XboxController adaptiveGamepad = new XboxController(ControllerConstants.kAdaptiveControllerPort); // Adaptive

	public OI() {

	}

	private static double deadBand(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public static double getDriverLeftX() {
		return deadBand(driverGamepad.getLeftX(), ControllerConstants.kDriverDeadBandLeftX);
	}

	public static double getDriverRightX() {
		return deadBand(driverGamepad.getRightX(), ControllerConstants.kDriverDeadBandRightX);
	}

	public static double getDriverLeftY() {
		return deadBand(driverGamepad.getLeftY(), ControllerConstants.kDriverDeadBandLeftY);
	}

	public static double getDriverRightY() {
		return deadBand(driverGamepad.getRightY(), ControllerConstants.kDriverDeadBandRightY);
	}

	// public static double getOperatorLeftX() {
	// 	return deadBand(operatorGamepad.getLeftX(), ControllerConstants.kOperatorDeadBandLeftX);
	// }

	// public static double getOperatorRightX() {
	// 	return deadBand(operatorGamepad.getRightX(), ControllerConstants.kOperatorDeadBandRightX);
	// }

	// public static double getOperatorLeftY() {
	// 	return deadBand(operatorGamepad.getLeftY(), ControllerConstants.kOperatorDeadBandLeftY);
	// }

	// public static double getOperatorRightY() {
	// 	return deadBand(operatorGamepad.getRightY(), ControllerConstants.kOperatorDeadBandRightY);
	// }

	// public static double getClimberLeftY() {
	// 	return deadBand(climberGamepad.getLeftY(), ControllerConstants.kClimberDeadBandLeftY);

	// }

	// public static double getClimberRightY() {
	// 	return deadBand(climberGamepad.getRightY(), ControllerConstants.kClimberDeadBandRightY);
	// }

	public static void configureButtonBindings(DriveSubsystem m_robotDrive) {

		//DRIVER//
		// Drive at half speed when the right bumper is held

		// new JoystickButton(driverGamepad, Button.kRightBumper.value)
		// 		.whenPressed(() -> /*Command*/)
		// 		.whenReleased(() -> /*Command*/);

		// new JoystickButton(driverGamepad, Button.kLeftBumper.value)
		// 		.whenPressed(() -> /*Command*/)
		// 		.whenReleased(() -> /*Command*/);

		// new JoystickButton(driverGamepad, Button.kY.value)
		// 		.whenPressed(new /*Command*/);

		new JoystickButton(driverGamepad, Button.kA.value)
				.whenPressed(new ResetGyroCommand(m_robotDrive));		
				
		// new JoystickButton(driverGamepad, Button.kBack.value)
		// 		.whenPressed(new /*Command*/)
		// 		.whenPressed(new /*Command*/);
		
		// new JoystickButton(driverGamepad, Button.kX.value)
		// 		.whenPressed(new /*Command*/);

		// new JoystickButton(driverGamepad, Button.kB.value)
		// 		.whileHeld(new /*Command*/)
		// 		.whenReleased(new /*Command*/);
				
		// new JoystickButton(driverGamepad, Button.kStart.value)
		// 		.whenPressed(new /*Command*/);
		
		// new JoystickButton(driverGamepad, Button.kRightStick.value)
		// 		.whenPressed(new /*Command*/);
				
		// new JoystickButton(driverGamepad, Button.kLeftStick.value)
		// 		.whenPressed(new /*Command*/);
			
		// new DPadButton(driverGamepad, DPadButton.Direction.UP)
		// 		.whenPressed(new /*Command*/);

		// new DPadButton(driverGamepad, DPadButton.Direction.DOWN)
		// 		.whileHeld(new /*Command*/);


		//OPERATOR//
				
		
		// new JoystickButton(operatorGamepad, Button.kLeftBumper.value)
		// 		.whenPressed(new /*Command*/);
		
		// new JoystickButton(operatorGamepad, Button.kRightBumper.value)
		// 		.whenPressed(new /*Command*/);
		
		// new JoystickButton(operatorGamepad, Button.kB.value)
		// 		.whenPressed(new /*Command*/)
		// 		.whenPressed(new /*Command*/);

		// new JoystickButton(operatorGamepad, Button.kA.value)
		// 		.whenPressed(new /*Command*/)
		// 		.whenReleased(new /*Command*/);

		// new JoystickButton(operatorGamepad, Button.kX.value)
		// 		.whenPressed(new /*Command*/)
		// 		.whenReleased(new /*Command*/);

		// new JoystickButton(operatorGamepad, Button.kRightStick.value)
		// 		.whenPressed(new /*Command*/);
				
		// new JoystickButton(operatorGamepad, Button.kLeftStick.value)
		// 		.whenPressed(new /*Command*/);	
				
		// new JoystickButton(operatorGamepad, Button.kBack.value)
		// 		.whileHeld(new /*Command*/);
				
		// new JoystickButton(operatorGamepad, Button.kStart.value)
		// 		.whenPressed(new /*Command*/);


		//Climber//

		// new JoystickButton(climberGamepad, Button.kY.value)
		// 	.whenPressed(new /*Command*/);

		// new JoystickButton(climberGamepad, Button.kA.value)
		// 	.whenPressed(new /*Command*/);
		
		// new JoystickButton(climberGamepad, Button.kStart.value)
		// 	.whenPressed(new /*Command*/);
		
		// new JoystickButton(climberGamepad, Button.kBack.value)
		// 	.whenPressed(new /*Command*/);
		
		// new JoystickButton(climberGamepad, Button.kB.value)
		// 	.whenPressed(new /*Command*/);	
		
		// new JoystickButton(climberGamepad, Button.kX.value)
		// 	.whenPressed(new /*Command*/);	
		
		// new JoystickButton(climberGamepad, Button.kRightBumper.value)
		// 	.whenPressed(new /*Command*/);
		
		//Adaptive Controller

		// new JoystickButton(adaptiveGamepad, Button.kStart.value)
		// 	.whenPressed(new /*Command*/);
		
		// new JoystickButton(adaptiveGamepad, Button.kBack.value)
		// 	.whenPressed(new /*Command*/);
			
		// new JoystickButton(adaptiveGamepad, Button.kB.value)
		// 	.whenPressed(new /*Command*/);
		
		// new JoystickButton(adaptiveGamepad, Button.kA.value)
		// 	.whenPressed(new /*Command*/);
	}
}