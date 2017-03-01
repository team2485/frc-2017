package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class DriveStraight extends Command {
	private double dist, angle;
	private double maxVelocity;
	private boolean finished;
	public DriveStraight(double dist, double angle, double maxVelocity) {
		this.dist = dist;
		this.angle = angle;
		this.maxVelocity =  maxVelocity;
		requires(RobotMap.driveTrain);
	}
	
	@Override
	protected void initialize() {
		super.initialize();
	}
	@Override
	protected void execute() {
		super.execute();
		finished = RobotMap.driveTrain.driveTo(dist, maxVelocity, angle, 0);
	}

	@Override
	protected boolean isFinished() {
		return finished;
	}

}
