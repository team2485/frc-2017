package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetLights extends InstantCommand {
	private Value val;
	public SetLights(Value val) {
		this.val = val;
	}
	@Override
	protected void initialize() {
		super.initialize();
		RobotMap.lightSpike.set(val);

	}
}

