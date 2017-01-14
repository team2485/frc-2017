package org.usfirst.frc.team2485.commands;

import org.usfirst.frc.team2485.robot.OI;

import edu.wpi.first.wpilibj.command.Command;

/**
* @author Nicholas Contreras
*/

public class DriveWithControllers extends Command {
	
	public public DriveWithControllers() {
	}
	
	@Override
	protected void initialize() {
	}
	
	@Override
	protected void execute() {
		double foward = -OI.xBox.getRawAxis(1);
		double right = OI.xBox.getRawAxis(4);
	}
	
	

	@Override
	protected boolean isFinished() {
		return false;
	}

}
