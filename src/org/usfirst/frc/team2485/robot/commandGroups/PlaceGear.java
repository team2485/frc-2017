package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.EjectGear;
import org.usfirst.frc.team2485.subsystems.SetIntakeArm;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * @author Ben Dorsey
 */

public class PlaceGear extends CommandGroup {
	public PlaceGear() {
		addSequential(new EjectGear());
		addSequential(new SetIntakeArm(20));
	}
}
