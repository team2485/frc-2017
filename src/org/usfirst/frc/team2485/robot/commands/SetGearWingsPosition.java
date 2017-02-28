package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetGearWingsPosition extends InstantCommand {
	
	private boolean open;
	
	public SetGearWingsPosition(boolean open) {
		this.open = open;
	}
	
	@Override
	protected void initialize() {
		RobotMap.gearHolder.setWingsOpen(open);
		System.out.println("open gear");
	}
}
