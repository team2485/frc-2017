package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * @author Ben Dorsey
 */


public class ToggleCompressor extends InstantCommand {
	
	private boolean on;
	
	public ToggleCompressor(boolean on) {
		this.on = on;
	}
	
	public void initialize() {
		if (on) {
			RobotMap.compressor.start();
		} else {
			RobotMap.compressor.stop();
		}
	}

}
