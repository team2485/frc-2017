package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetFeederManual extends InstantCommand {
	
	private double pwm;
	
	public SetFeederManual (double pwm) {
		this.pwm = pwm;
	}
	
	protected void initialize() {
		RobotMap.feeder.setManual(pwm);
	}
}
