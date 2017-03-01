package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.AutoPath;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class DriveTo extends Command {
	private AutoPath path;
	private double maxVelocity;
	private boolean finished, reverse;
	public DriveTo(AutoPath path, double maxVelocity, boolean reverse) {
		this.path = path;
		this.maxVelocity =  maxVelocity;
		this.reverse = reverse;
		requires(RobotMap.driveTrain);
	}
	
	@Override
	protected void initialize() {
		super.initialize();
		RobotMap.driveTrain.zeroEncoders();
	}
	@Override
	protected void execute() {

		double arcLength = RobotMap.averageEncoderDistance.pidGet(), 
				pathLength = path.getPathLength();
		
		if (reverse) {
			arcLength = pathLength - arcLength;
			pathLength *= -1;
		}
		
		finished = RobotMap.driveTrain.driveTo(pathLength, maxVelocity, 
				path.getHeadingAtDist(arcLength), path.getCurvatureAtDist(arcLength));
		
	}

	@Override
	protected boolean isFinished() {
		return finished;
	}

}
