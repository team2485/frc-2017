package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class DriveStraight extends Command {
	private double dist, angle;
	private double maxVelocity;
	private boolean finished;
	private int timeout;
	private long startTime;
	public DriveStraight(double dist, double angle, double maxVelocity, int timeout) {
		this.dist = dist;
		this.angle = angle;
		this.maxVelocity =  maxVelocity;
		this.timeout = timeout;
		requires(RobotMap.driveTrain);
	}
	
	@Override
	protected void initialize() {
		super.initialize();
		startTime = System.currentTimeMillis();
	}
	@Override
	protected void execute() {
		super.execute();
		finished = RobotMap.driveTrain.driveTo(dist, maxVelocity, angle, 0);
	}

	@Override
	protected boolean isFinished() {
		return finished || (System.currentTimeMillis() - startTime) > timeout;
	}
	
	@Override
	protected void end() {
		super.end();

	}

}
