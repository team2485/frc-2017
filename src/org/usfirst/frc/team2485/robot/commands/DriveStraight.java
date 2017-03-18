package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.subsystems.DriveTrain;

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
	private double tolerance;
	private boolean setAngle = false;
	private FinishedCondition finishedCondition = FinishedCondition.FALSE_CONDITION;
	
	public DriveStraight(double dist, double maxVelocity, int timeout) {
		this(dist, 0, maxVelocity, timeout);
		setAngle = true;
	}
	
	public DriveStraight(double dist, double angle, double maxVelocity, int timeout) {
		this(dist, angle, maxVelocity, timeout, DriveTrain.DRIVETO_TOLERANCE);
	}	
	
	public DriveStraight(double dist, double angle, double maxVelocity, int timeout, double tolerance) {
		this.dist = dist;
		this.angle = angle;
		this.maxVelocity =  maxVelocity;
		this.timeout = timeout;
		this.tolerance = tolerance;
		requires(RobotMap.driveTrain);
	}
	
	public void setFinishedCondition(FinishedCondition finishedCondition) {
		this.finishedCondition = finishedCondition;
	}
	
	@Override
	protected void initialize() {
		super.initialize();
		RobotMap.driveTrain.zeroEncoders();
		startTime = System.currentTimeMillis();
		if (setAngle)
			angle = RobotMap.ahrs.getAngle();
	}
	@Override
	protected void execute() {
		super.execute();
		finished = RobotMap.driveTrain.driveTo(dist, maxVelocity, angle, 0, tolerance) ||
				finishedCondition.isFinished();
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
