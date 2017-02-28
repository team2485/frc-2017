package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ResetGear extends InstantCommand {
	public ResetGear() {
		requires(RobotMap.gearHolder);
	}
	@Override
	protected void initialize() {
		RobotMap.gearHolder.setFlapsOpen(false);
		RobotMap.gearHolder.setWingsOpen(false);
	}
}
