package org.usfirst.frc.team2485.robot.commands;


import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;


public class RollersOn extends InstantCommand {
	
	private double power;

	public RollersOn(double pow) {
		requires(RobotMap.intake);
		this.power = pow;
	}
	
	@Override
	protected void initialize() {
		RobotMap.intake.setManual(power);
	}

}
