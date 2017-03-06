package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * @author Ben Dorsey
 */

public class CheckDistError extends InstantCommand {
	@Override
	protected void initialize() {
		RobotMap.driveTrain.setAutoError();
	}
}
