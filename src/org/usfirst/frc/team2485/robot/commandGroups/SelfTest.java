package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.selftest.TestAHRSDriftRate;
import org.usfirst.frc.team2485.robot.commands.selftest.TestDriveMotorDirections;
import org.usfirst.frc.team2485.robot.commands.selftest.TestSWODCurrentDraw;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
* @author Nicholas Contreras
*/

public class SelfTest extends CommandGroup {
	
	public SelfTest() {
		addSequential(new TestDriveMotorDirections());
		addSequential(new TestAHRSDriftRate());
		addSequential(new TestSWODCurrentDraw());
	}
}
