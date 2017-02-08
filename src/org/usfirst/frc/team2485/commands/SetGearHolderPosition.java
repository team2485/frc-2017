package org.usfirst.frc.team2485.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetGearHolderPosition extends InstantCommand {
	
	private boolean open;
	
	public SetGearHolderPosition(boolean open) {
		this.open = open;
	}
	
	@Override
	protected void initialize() {
		RobotMap.gearHolder.setOpen(open);
	}
}
