package org.usfirst.frc.team2485.robot.commands.selftest;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.commandGroups.SelfTest;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * @author Nicholas Contreras
 */

public class PrepForSelfTest extends Command {

	private long startTime;

	private boolean done;

	@Override
	protected void initialize() {
		startTime = System.currentTimeMillis();
		done = false;
	}

	@Override
	protected void execute() {
		
		if (!OI.xBox.getRawButton(OI.XBOX_BTN_BACK) || !OI.xBox.getRawButton(OI.XBOX_BTN_START)) {
			done = true;
		}

		if (System.currentTimeMillis() - startTime > 3000) {
			Scheduler.getInstance().add(new SelfTest());
			done = true;
		}
	}

	@Override
	protected boolean isFinished() {
		return done;
	}
}
