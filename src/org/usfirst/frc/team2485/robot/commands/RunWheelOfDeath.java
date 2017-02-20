package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;
/**
 * 
 * @author Vicky
 *
 */

public class RunWheelOfDeath extends InstantCommand {
	private double current;

	public RunWheelOfDeath(double current) {
		this.current = current;
	}
	
	@Override
	protected void initialize() {
		RobotMap.wheelOfDeath.setCurrent(current);
	}
	
}
