package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class RotateTo extends Command{
	private double angle;
	private long timeout;
	private boolean finished;
	private long startTime;
	
	public RotateTo(double angle, long timeout) {
		this.angle = angle;
		this.timeout = timeout;
	}
	
	@Override
	protected void initialize() {
		startTime = System.currentTimeMillis();
	}
	
	@Override
	protected void execute() {
		finished = RobotMap.driveTrain.rotateTo(angle);
	}
	
	@Override
	protected boolean isFinished() {
		return finished || (System.currentTimeMillis() - startTime) > timeout;
	}

}
