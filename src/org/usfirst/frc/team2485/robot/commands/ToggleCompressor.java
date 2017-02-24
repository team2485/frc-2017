package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * @author Ben Dorsey
 */


public class ToggleCompressor extends InstantCommand {
	
	private boolean state;
	
	public ToggleCompressor(boolean state) {
		this.state = state;
	}
	
	public void initialize() {
		if (state) {
			RobotMap.compressor.start();
		} else {
			RobotMap.compressor.stop();
		}
	}

}
