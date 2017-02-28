package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * @author Ben Dorsey
 */

public class SetSWODSpeed extends InstantCommand {
	private double speed;
	public SetSWODSpeed(double speed) {
		this.speed = speed;
	}
	@Override
	protected void initialize() {
		RobotMap.wheelOfDeath.setSpeed(speed);
	}
}
