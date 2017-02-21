package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class HighLowCurrentTest extends Command {
	
	private double highLeft, lowLeft, highRight, lowRight;
	private long period, startTime;
	private double scaleCIM, scale775;
	
	/**
	 * Alternates high and low current
	 * @param highLeft high current for left side in amps
	 * @param lowLeft low current for left side in amps
	 * @param highRight high current for right side in amps
	 * @param lowRight low current for right side in amps
	 * @param period time to cycle in millis
	 */
	public HighLowCurrentTest(double highLeft, double lowLeft, double highRight, double lowRight, long period, double scaleCIM, double scale775) {
		this.highLeft = highLeft;
		this.highRight = highRight;
		this.lowLeft = lowLeft;
		this.lowRight = lowRight;
		this.period = period;
		this.scaleCIM = scaleCIM;
		this.scale775 = scale775;
	}
	
	@Override
	protected void initialize() {
		startTime = System.currentTimeMillis();
	}
	
	@Override
	protected void execute() {
		long cycleTime = (System.currentTimeMillis() - startTime) % period;
		if (cycleTime > period / 2) { // low
			RobotMap.driveTrain.setLeftRightCurrent(lowLeft*scaleCIM, lowRight*scaleCIM, lowLeft*scale775, lowRight*scale775);
		} else { // high
			RobotMap.driveTrain.setLeftRightCurrent(highLeft*scaleCIM, highRight*scaleCIM, highLeft*scale775, highRight*scale775);
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

}
