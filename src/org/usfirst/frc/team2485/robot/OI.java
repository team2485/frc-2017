package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commands.Climb;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.robot.commands.RunWheelOfDeath;
import org.usfirst.frc.team2485.robot.commands.SetDriveSpeed;
import org.usfirst.frc.team2485.robot.commands.SetGearChutePosition;
import org.usfirst.frc.team2485.robot.commands.SetGearHolderPosition;
import org.usfirst.frc.team2485.robot.commands.SetQuickTurn;
import org.usfirst.frc.team2485.robot.commands.selftest.PrepForSelfTest;
import org.usfirst.frc.team2485.subsystems.DriveTrain.DriveSpeed;
import org.usfirst.frc.team2485.util.JoystickAxisButton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.networktables.NetworkTablesJNI;

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
	public static final int XBOX_BTN_LBUMPER = 5;
	public static final int XBOX_BTN_RBUMPER = 6;
	public static final int XBOX_BTN_BACK = 7;
	public static final int XBOX_BTN_START = 8;
	public static final int XBOX_BTN_L_AXIS = 9;

	public static final int XBOX_AXIS_LX = 0;
	public static final int XBOX_AXIS_LY = 1;
	public static final int XBOX_AXIS_LTRIGGER = 2;
	public static final int XBOX_AXIS_RTRIGGER = 3;
	public static final int XBOX_AXIS_RX = 4;
	public static final int XBOX_AXIS_RY = 5;

	public static void init() {
		xBox = new Joystick(0);
		joystick = new Joystick(1);

		new BackStartComboButton().whenPressed(new PrepForSelfTest());

		new JoystickButton(xBox, XBOX_BTN_LBUMPER).whileHeld(new Climb());

		new JoystickButton(xBox, XBOX_BTN_A).whenPressed(new SetGearHolderPosition(true));

		if (DriveWithControllers.TRIGGER_DRIVE) {
			new JoystickButton(xBox, XBOX_BTN_X).whenPressed(new SetQuickTurn(true));
			new JoystickButton(xBox, XBOX_BTN_X).whenReleased(new SetQuickTurn(false));
		} else {
			new JoystickButton(xBox, XBOX_BTN_RBUMPER).whenPressed(new SetQuickTurn(true));
			new JoystickButton(xBox, XBOX_BTN_RBUMPER).whenReleased(new SetQuickTurn(false));

			new JoystickAxisButton(xBox, XBOX_AXIS_RTRIGGER, 0.4, 1)
					.whenPressed(new SetDriveSpeed(DriveSpeed.SLOW_SPEED_RATING));
			new JoystickAxisButton(xBox, XBOX_AXIS_RTRIGGER, 0.4, 1)
					.whenReleased(new SetDriveSpeed(DriveSpeed.NORMAL_SPEED_RATING));
		}

//		new JoystickButton(joystick, 7).whenPressed(new SetGearChutePosition(true));
//		new JoystickButton(joystick, 8).whenPressed(new SetGearChutePosition(false));
//		new JoystickButton(joystick, 10).whenPressed(new SetGearHolderPosition(false));
		
		new LogitechKeypadButton('1').whenPressed(new SetGearChutePosition(true));
		new LogitechKeypadButton('2').whenPressed(new SetGearChutePosition(false));
		new LogitechKeypadButton('8').whenPressed(new SetGearHolderPosition(false));
	}

	/**
	 * Combines the Start and Back buttons on the XBOX controller into one
	 * button that requires both pressed to be valid
	 */
	private static class BackStartComboButton extends Button {
		@Override
		public boolean get() {
			return xBox.getRawButton(XBOX_BTN_BACK) && xBox.getRawButton(XBOX_BTN_START);
		}
	}
	private static class LogitechKeypadButton extends Button {
		private char key; 
		
		public LogitechKeypadButton(char key) {
			this.key = key;
		}
		@Override
		public boolean get() {
			return NetworkTable.getTable("LogitechController").getBoolean(key + "", false);
		}
		
	}
}