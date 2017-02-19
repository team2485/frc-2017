package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetLeftRightVelocity extends InstantCommand {
	
	private double l, r;
	
	public SetLeftRightVelocity(double l, double r) {
		this.l = l;
		this.r = r;
	}
	
	@Override
	protected void initialize() {
		RobotMap.driveTrain.setLeftRightVelocity(l, r);
	}
}
