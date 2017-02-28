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
	private boolean finished;
	public DriveTo(AutoPath path, double maxVelocity) {
		this.path = path;
		this.maxVelocity =  maxVelocity;
		requires(RobotMap.driveTrain);
	}
	
	@Override
	protected void initialize() {
		super.initialize();
		RobotMap.driveTrain.zeroEncoders();
	}
	@Override
	protected void execute() {
		super.execute();
		double arcLength = RobotMap.averageEncoderDistance.pidGet();
		finished = RobotMap.driveTrain.driveTo(path.getPathLength(), maxVelocity, path.getHeadingAtDist(arcLength), path.getCurvatureAtDist(arcLength));
	}

	@Override
	protected boolean isFinished() {
		return finished;
	}

}
