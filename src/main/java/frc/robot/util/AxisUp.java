package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;


/**
 *	Makes moving an axis up on a joystick result in a button input
 */
public class AxisUp extends Trigger {
    
	private XboxController gamepad;
	private int axis;
	
	public AxisUp(XboxController targetGamepad, int targetAxis) {
		gamepad = targetGamepad;
		axis = targetAxis;
	}
	
    public boolean get() {
        return (gamepad.getRawAxis(axis) >= .1);
    }
}
