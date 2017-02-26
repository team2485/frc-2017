package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * @author Ben Dorsey
 */

public class SetShooterManual extends InstantCommand {
	
	private double pwm;
	
	public SetShooterManual(double pwm) {
		requires(RobotMap.shooter);
		this.pwm = pwm;
	}
	
	@Override
	protected void initialize() {
		RobotMap.shooter.setManual(pwm);
	}

}
