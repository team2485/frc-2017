package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetGearFlapsPosition extends InstantCommand {
	
	private boolean open;
	
	public SetGearFlapsPosition(boolean open) {
		requires(RobotMap.gearHolder);
		this.open = open;
	}
	
	@Override
	protected void initialize() {
		RobotMap.gearHolder.setFlapsOpen(open);
	}
	
}
