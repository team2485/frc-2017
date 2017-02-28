package org.usfirst.frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class Pause extends Command{
	private int timeout; 
	private long startTime;
	
	public Pause(int timeout) {
		this.timeout = timeout;
	}
	
	@Override
	protected void initialize() {
		startTime = System.currentTimeMillis();
	}

	@Override
	protected boolean isFinished() {
		return ((System.currentTimeMillis() - startTime) > timeout);
	}

}
