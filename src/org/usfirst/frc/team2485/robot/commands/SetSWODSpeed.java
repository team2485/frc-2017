package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class SetSWODSpeed extends Command {
	
	public SetSWODSpeed () {
		requires(RobotMap.wheelOfDeath);
		setInterruptible(true);
	}
	
	
	@Override
	protected void execute() {
		RobotMap.wheelOfDeath.setSpeed(ThresholdHandler.deadbandAndScale(OI.elliot.getRawAxis(OI.XBOX_AXIS_RTRIGGER), 
				0.1, 0.5, 1));
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}

}
