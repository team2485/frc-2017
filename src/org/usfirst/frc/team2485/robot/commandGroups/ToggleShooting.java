package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.Pause;
import org.usfirst.frc.team2485.robot.commands.SetFeederManual;
import org.usfirst.frc.team2485.robot.commands.SetFeederSpeed;
import org.usfirst.frc.team2485.robot.commands.SetSWODManual;
import org.usfirst.frc.team2485.robot.commands.SetSWODSpeed;
import org.usfirst.frc.team2485.robot.commands.SetShooterManual;
import org.usfirst.frc.team2485.robot.commands.SpinUpShooter;
import org.usfirst.frc.team2485.robot.commands.StopShooter;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * @author Ben Dorsey
 */

public class ToggleShooting extends CommandGroup {
	
	public ToggleShooting(boolean on) {
		if (on) {
			addSequential(new SetSWODSpeed());
		} else {
			addSequential(new SetSWODManual(0));
		}
	}
	
}
