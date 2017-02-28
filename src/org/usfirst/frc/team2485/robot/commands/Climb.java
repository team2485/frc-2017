package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class Climb extends InstantCommand {
	double power; 
	
	public Climb(double power) {
		requires(RobotMap.climber);
		setInterruptible(true);
		this.power = power;
	}
	
	@Override
	public void initialize() {
		RobotMap.climber.setPower(power);
	}
	
	@Override
	protected void interrupted() {
		end();
	}
	
	@Override
	protected void end() {
//		RobotMap.climber.stopClimbing();
	}
}
