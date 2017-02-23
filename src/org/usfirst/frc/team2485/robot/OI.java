package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commandGroups.ResetGear;
import org.usfirst.frc.team2485.robot.commandGroups.SetIntake;
import org.usfirst.frc.team2485.robot.commands.Climb;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.robot.commands.SetIntakeArmHorizontal;
import org.usfirst.frc.team2485.robot.commands.SetRollers;
import org.usfirst.frc.team2485.robot.commands.RunWheelOfDeath;
import org.usfirst.frc.team2485.robot.commands.SetDriveSpeed;
import org.usfirst.frc.team2485.robot.commands.SetGearChutePosition;
import org.usfirst.frc.team2485.robot.commands.SetGearHolderPosition;
import org.usfirst.frc.team2485.robot.commands.SetQuickTurn;
import org.usfirst.frc.team2485.robot.commands.TestVerticalSolenoid1;
import org.usfirst.frc.team2485.robot.commands.TestVerticalSolenoid2;
import org.usfirst.frc.team2485.robot.commands.selftest.PrepForSelfTest;
import org.usfirst.frc.team2485.subsystems.DriveTrain.DriveSpeed;
import org.usfirst.frc.team2485.util.JoystickAxisButton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public static Joystick ben;
	public static Joystick elliot;

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
		ben = new Joystick(0);
		elliot = new Joystick(1);

		new BackStartComboButton().whenPressed(new PrepForSelfTest());

		new JoystickButton(ben, XBOX_BTN_LBUMPER).whenPressed(new Climb(1));
		new JoystickButton(ben, XBOX_BTN_LBUMPER).whenReleased(new Climb(0));

		new JoystickButton(ben, XBOX_BTN_A).whenPressed(new SetGearHolderPosition(true));
		
		new JoystickAxisButton(elliot, XBOX_AXIS_RTRIGGER, .2, 1).whenPressed(new RunWheelOfDeath(true));
		new JoystickAxisButton(elliot, XBOX_AXIS_RTRIGGER, .2, 1).whenReleased(new RunWheelOfDeath(false));


		if (DriveWithControllers.TRIGGER_DRIVE) {
			new JoystickButton(ben, XBOX_BTN_X).whenPressed(new SetQuickTurn(true));
			new JoystickButton(ben, XBOX_BTN_X).whenReleased(new SetQuickTurn(false));
		} else {
			new JoystickButton(ben, XBOX_BTN_RBUMPER).whenPressed(new SetQuickTurn(true));
			new JoystickButton(ben, XBOX_BTN_RBUMPER).whenReleased(new SetQuickTurn(false));

			new JoystickAxisButton(ben, XBOX_AXIS_RTRIGGER, 0.4, 1)
					.whenPressed(new SetDriveSpeed(DriveSpeed.SLOW_SPEED_RATING));
			new JoystickAxisButton(ben, XBOX_AXIS_RTRIGGER, 0.4, 1)
					.whenReleased(new SetDriveSpeed(DriveSpeed.NORMAL_SPEED_RATING));
		}

		new JoystickButton(elliot, XBOX_BTN_A).whenPressed(new SetGearChutePosition(true));
		new JoystickButton(elliot, XBOX_BTN_B).whenPressed(new ResetGear());
		new JoystickAxisButton(elliot, XBOX_AXIS_LTRIGGER, 0.2, 1).whenPressed(new SetRollers(true));
		new JoystickButton(elliot, XBOX_BTN_LBUMPER).whenPressed(new SetRollers(false));
		 
		new JoystickButton(elliot, XBOX_BTN_X).whenPressed(new SetIntakeArmHorizontal(true));
		new JoystickButton(elliot, XBOX_BTN_X).whenReleased(new SetIntakeArmHorizontal(false));
		new JoystickButton(elliot, XBOX_BTN_RBUMPER).whenPressed(new TestVerticalSolenoid1(true));
		new JoystickButton(elliot, XBOX_BTN_RBUMPER).whenReleased(new TestVerticalSolenoid1(false));
		new JoystickAxisButton(elliot, XBOX_AXIS_RTRIGGER, .2, 1).whenPressed(new TestVerticalSolenoid2(true));
		new JoystickAxisButton(elliot, XBOX_AXIS_RTRIGGER, .2, 1).whenReleased(new TestVerticalSolenoid2(false));

		new JoystickButton(elliot, XBOX_BTN_Y).whenPressed(new SetIntake());
		
//		new LogitechKeypadButton('1').whenPressed(new SetGearChutePosition(true));
//		new LogitechKeypadButton('2').whenPressed(new SetGearChutePosition(false));
//		new LogitechKeypadButton('8').whenPressed(new SetGearHolderPosition(false));
	}

	/**
	 * Combines the Start and Back buttons on the XBOX controller into one
	 * button that requires both pressed to be valid
	 */
	private static class BackStartComboButton extends Button {
		@Override
		public boolean get() {
			return ben.getRawButton(XBOX_BTN_BACK) && ben.getRawButton(XBOX_BTN_START);
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