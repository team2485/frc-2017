package org.usfirst.frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * @author Ben Dorsey
 */

public class CancelCommand extends InstantCommand {
	private Command c;
	public CancelCommand(Command c) {
		this.c = c;
	}
	@Override
	protected void initialize() {
		super.initialize();
		c.cancel();
	}
}
