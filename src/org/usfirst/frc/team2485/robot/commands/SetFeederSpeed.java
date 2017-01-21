package org.usfirst.frc.team2485.robot.commands;


import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetFeederSpeed extends InstantCommand {
	private double rps;

	public SetFeederSpeed(double rps) {
		requires(RobotMap.feeder);
		this.rps = rps;
	}

	@Override
	protected void initialize() {
		RobotMap.feeder.setTargetSpeed(rps);
	}

}
