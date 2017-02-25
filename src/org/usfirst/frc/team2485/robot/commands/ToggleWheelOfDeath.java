package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 * @author Vicky
 *
 */

public class ToggleWheelOfDeath extends Command {

	private boolean on;

	public ToggleWheelOfDeath(boolean on) {
		requires(RobotMap.wheelOfDeath);
		setInterruptible(true);
		this.on = on;
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
		if (System.currentTimeMillis()
				% (ConstantsIO.kSWODForwardTime + ConstantsIO.kSWODReverseTime) <= ConstantsIO.kSWODForwardTime) {
			RobotMap.wheelOfDeath.setCurrent(ConstantsIO.kSWODCurrent);
		} else {
			RobotMap.wheelOfDeath.setCurrent(-ConstantsIO.kSWODCurrent);
		}
	}

	@Override
	protected void interrupted() {
		end();
	}

	@Override
	protected void end() {
		RobotMap.wheelOfDeath.stop();
	}

	@Override
	protected boolean isFinished() {
		return !on;
	}
}
