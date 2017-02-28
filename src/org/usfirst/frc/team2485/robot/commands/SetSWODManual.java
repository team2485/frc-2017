package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class SetSWODManual extends Command {
	private double pwm;
	
	
	public SetSWODManual(double pwm) {
		this.pwm = pwm;
		requires(RobotMap.wheelOfDeath);
	}

	
	@Override
	protected void execute() {
		RobotMap.wheelOfDeath.setPWM(pwm);
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
	
	
}
