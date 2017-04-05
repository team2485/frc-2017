package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class Rumble extends Command {
	private long startTime, timeout;
	private double rumbleLeft, rumbleRight;
	public Rumble(double rumbleLeft, double rumbleRight, long timeout) {
		this.timeout = timeout;
		this.rumbleLeft = rumbleLeft;
		this.rumbleRight = rumbleRight;
	}
	
	@Override
	protected void initialize() {
		super.initialize();
		startTime = System.currentTimeMillis();
		OI.benRumble.setRumble(RumbleType.kLeftRumble, rumbleLeft);
		OI.benRumble.setRumble(RumbleType.kRightRumble, rumbleRight);
	}
	
	@Override
	protected boolean isFinished() {
		return System.currentTimeMillis() > startTime + timeout;
	}
	
	@Override
	protected void end() {
		super.end();
		OI.benRumble.setRumble(RumbleType.kLeftRumble, 0);
		OI.benRumble.setRumble(RumbleType.kRightRumble, 0);
	}

}
