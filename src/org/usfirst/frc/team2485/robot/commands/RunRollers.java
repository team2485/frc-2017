package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * @author Ben Dorsey
 */

public class RunRollers extends Command {
	private static final double PWM = .8;
	private static final double CURRENT_THRESHOLD = 20;
	private static final double MIN_TIME = 1500;
	private static double startTime, stallTime;
	
	@Override
	protected void initialize() {
		RobotMap.gearIntakeRoller.setManual(PWM);
		startTime = System.currentTimeMillis();
	}
	
	protected void end() {
			RobotMap.gearIntakeRoller.setManual(0);
	}

	@Override
	protected boolean isFinished() {
		if ((System.currentTimeMillis() - startTime) < MIN_TIME) {
			return false;
		} else {
			return RobotMap.gearIntakeRoller.getCurrent() > CURRENT_THRESHOLD;
		}
	}

}
