package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class HighLowCurrentTest extends Command {
	
	private double highLeft, lowLeft, highRight, lowRight;
	private long period, startTime;
	
	/**
	 * Alternates high and low current
	 * @param highLeft high current for left side in amps
	 * @param lowLeft low current for left side in amps
	 * @param highRight high current for right side in amps
	 * @param lowRight low current for right side in amps
	 * @param period time to cycle in millis
	 */
	public HighLowCurrentTest(double highLeft, double lowLeft, double highRight, double lowRight, long period) {
		this.highLeft = highLeft;
		this.highRight = highRight;
		this.lowLeft = lowLeft;
		this.lowRight = lowRight;
		this.period = period;
	}
	
	@Override
	protected void initialize() {
		startTime = System.currentTimeMillis();
	}
	
	@Override
	protected void execute() {
		long cycleTime = (System.currentTimeMillis() - startTime) % period;
		if (cycleTime > period / 2) { // low
			RobotMap.driveTrain.setLeftRightCurrent(lowLeft, lowRight);
		} else { // high
			RobotMap.driveTrain.setLeftRightCurrent(highLeft, highRight);
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

}
