package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.ZeroGearIntakeEncoder;
import org.usfirst.frc.team2485.subsystems.LowerGearIntakeArm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * @author Ben Dorsey
 */

public class InitializeIntakeArm extends CommandGroup {
	public InitializeIntakeArm() {
		addSequential(new LowerGearIntakeArm(1000));
		addSequential(new TimedCommand(2));
		addSequential(new ZeroGearIntakeEncoder());
	}
}
