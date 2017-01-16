package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.commands.SetQuickTurn;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
    public static Joystick xBox;
    public static Joystick joystick;
    
    public static final int XBOX_BTN_A = 1;
    public static final int XBOX_BTN_B = 2;
    public static final int XBOX_BTN_X = 3;
    public static final int XBOX_BTN_Y = 4;
    public static final int XBOX_LBUMPER = 5;
    public static final int XBOX_RBUMPER = 6;
    public static final int XBOX_LTRIGGER = 2;
    public static final int XBOX_RTRIGGER = 3;
    
    public static void init(){
    	xBox = new Joystick(0);
    	joystick = new Joystick(1);
    	
    	new JoystickButton(xBox, XBOX_RBUMPER).whenPressed(new SetQuickTurn(true));
    	new JoystickButton(xBox, XBOX_RBUMPER).whenReleased(new SetQuickTurn(false));
    }
    
    public static boolean getLeftTrigger() {
    	return xBox.getRawAxis(XBOX_LTRIGGER) > 0.4;
    }
    
    public static boolean getRightTrigger() {
    	return xBox.getRawAxis(XBOX_RTRIGGER) > 0.4;
    }
}

