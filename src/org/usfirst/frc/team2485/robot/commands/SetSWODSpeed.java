package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * @author Ben Dorsey
 */

public class SetSWODSpeed extends Command {

	private long lastJamTime;

	private static final int UNJAM_TIME = 135, REJAM_TIME = 250;
	public static final double NORMAL_POWER = .225, UNJAM_POWER = -.4;
	public static final double JAM_CURRENT_THRESHOLD = 2;

	public SetSWODSpeed() {
		requires(RobotMap.wheelOfDeath);
		setInterruptible(true);
	}

	@Override
	protected void execute() {
		long curTime = System.currentTimeMillis();
		if (curTime - lastJamTime < UNJAM_TIME) {
			RobotMap.wheelOfDeath.setPWM(UNJAM_POWER);
		} else if (curTime - lastJamTime < UNJAM_TIME + REJAM_TIME) {
			RobotMap.wheelOfDeath.setPWM(NORMAL_POWER);
		} else if (RobotMap.wheelOfDeath.getCurrent() > JAM_CURRENT_THRESHOLD) {
			lastJamTime = curTime;
		} else {
			RobotMap.wheelOfDeath.setPWM(NORMAL_POWER);
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

}
