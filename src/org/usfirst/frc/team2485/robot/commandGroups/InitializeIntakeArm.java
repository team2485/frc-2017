package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.LowerGearIntakeArm;
import org.usfirst.frc.team2485.robot.commands.ZeroGearIntakeEncoder;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * @author Ben Dorsey
 */

public class InitializeIntakeArm extends CommandGroup {
	public InitializeIntakeArm() {
		setInterruptible(false);
		addSequential(new LowerGearIntakeArm(1500));
		addSequential(new TimedCommand(.5));
		addSequential(new ZeroGearIntakeEncoder());
	}
}
