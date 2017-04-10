package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class StartDeadReckoning extends InstantCommand {
	public StartDeadReckoning() {
		setRunWhenDisabled(true);
	}
	@Override
	protected void initialize() {
		RobotMap.autoDeadReckoning.start();
	}
}
