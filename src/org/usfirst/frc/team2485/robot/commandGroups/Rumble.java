package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.ToggleRumble;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * @author Ben Dorsey
 */

public class Rumble extends CommandGroup {
	public Rumble(XboxController xbox, double rumbleLeft, double rumbleRight, double timeout) {
		addSequential(new ToggleRumble(xbox, rumbleLeft, rumbleRight));
		addSequential(new TimedCommand(timeout));
		addSequential(new ToggleRumble(xbox, 0, 0));
	}

}
