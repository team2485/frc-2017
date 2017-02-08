package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class Climb extends InstantCommand {
	
	public Climb() {
		requires(RobotMap.climber);
	}
	
	public void initialize() {
		RobotMap.climber.climb();
	}

}
