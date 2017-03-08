package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class WaitTillClose extends Command {
	private double dist;
	
	public WaitTillClose(double dist) {
		this.dist = dist;
	}
	
	@Override
	protected boolean isFinished() {
		return dist > Math.abs(RobotMap.driveTrain.getDistanceError());
	}

}
