package org.usfirst.frc.team2485.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetQuickTurn extends InstantCommand {
	
	private boolean quickTurn;
	
	public SetQuickTurn(boolean quickTurn) {
		this.quickTurn = quickTurn;
	}
	
	@Override
	protected void initialize() {
		RobotMap.driveTrain.setQuickTurn(quickTurn);
	}
}
