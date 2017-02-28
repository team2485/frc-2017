package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.SetFeederManual;
import org.usfirst.frc.team2485.robot.commands.SetFeederSpeed;
import org.usfirst.frc.team2485.robot.commands.SpinUpShooter;
import org.usfirst.frc.team2485.robot.commands.StopShooter;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * @author Ben Dorsey
 */

public class SetSpinUp extends CommandGroup {
	public SetSpinUp(boolean on) {
		if (on) {
			addParallel(new SpinUpShooter());
			addParallel(new SetFeederSpeed(50));
		} else {
			addParallel(new StopShooter());
			addParallel(new SetFeederManual(0));
		}
	}
}
