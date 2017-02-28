package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.SetSWODManual;
import org.usfirst.frc.team2485.robot.commands.SetSWODSpeed;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * @author Ben Dorsey
 */

public class ToggleShooting extends CommandGroup {
	
	public ToggleShooting(boolean on) {
		if (on) {
			addSequential(new SetSWODSpeed());
		} else {
			addSequential(new SetSWODManual(0));
		}
	}
	
}
