package org.usfirst.frc.team2485.robot.commands.selftest;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class TestGearDropper extends Command {
	
	private long startTime;
	private boolean done;
	private boolean gearDetected;
	
	public TestGearDropper() {
		requires(RobotMap.gearHolder);
	}
	
	@Override
	protected void initialize() {
		startTime = System.currentTimeMillis();
		done = false;
		RobotMap.gearHolder.setBottomOpen(false);
	}
	
	@Override
	protected void execute() {
		if (System.currentTimeMillis() - startTime > 500) {
			RobotMap.gearHolder.gearDetected();
		}
	}
	
	@Override
	protected boolean isFinished() {
		return done;
	}

}
