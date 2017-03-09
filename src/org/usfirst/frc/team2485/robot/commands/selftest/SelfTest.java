package org.usfirst.frc.team2485.robot.commands.selftest;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * @author Nicholas Contreras
 */

public class SelfTest extends CommandGroup {

	public SelfTest() {
		addSequential(new TestAHRSDriftRate());
		addSequential(new TestDriveMotorDirections());
		addSequential(new TestSWODCurrentDraw());
		addSequential(new TestShooterMotorDirections());
		addSequential(new TimedCommand(1));
		addSequential(new TestShooterSpeed());
	}
}
