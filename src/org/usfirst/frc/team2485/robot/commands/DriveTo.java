package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.util.AutoPath;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class DriveTo extends Command {
	private AutoPath path;
	private double maxVelocity;
	private boolean finished, reverse;
	private long startTime;
	private int timeout;
	private FinishedCondition finishedCondition = FinishedCondition.FALSE_CONDITION;
	public DriveTo(AutoPath path, double maxVelocity, boolean reverse, int timeout) {
		this.path = path;
		this.maxVelocity =  maxVelocity;
		this.reverse = reverse;
		this.timeout = timeout;
		setInterruptible(true);
		requires(RobotMap.driveTrain);
	}
	
	public void setFinishedCondition(FinishedCondition finishedCondition) {
		this.finishedCondition = finishedCondition;
	}
	
	@Override
	protected void initialize() {
		super.initialize();
		startTime = System.currentTimeMillis();
		RobotMap.driveTrain.zeroEncoders();
	}
	@Override
	protected void execute() {

		double arcLength = RobotMap.averageEncoderDistance.pidGet(), 
				pathLength = path.getPathLength();
		
		if (reverse) {
			arcLength = pathLength + arcLength;
			pathLength *= -1;
		}
		
		finished = RobotMap.driveTrain.driveTo(pathLength, maxVelocity, 
				path.getHeadingAtDist(arcLength), path.getCurvatureAtDist(arcLength), DriveTrain.DRIVETO_TOLERANCE) ||
				finishedCondition.isFinished();
		
	}
	
	@Override
	protected void interrupted() {
		finished = true;
	}
	
	@Override
	public synchronized void cancel() {
		finished = true;
	}

	@Override
	protected boolean isFinished() {
		
		return finished || (System.currentTimeMillis() - startTime) > timeout;
	}

}
