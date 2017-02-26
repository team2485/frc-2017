package org.usfirst.frc.team2485.robot.commands;


import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetFeederSpeed extends InstantCommand {
	private double ips;

	public SetFeederSpeed(double ips) {
		requires(RobotMap.feeder);
		this.ips = ips;
	}

	@Override
	protected void initialize() {
		RobotMap.feeder.setTargetSpeed(ips);
	}

}
