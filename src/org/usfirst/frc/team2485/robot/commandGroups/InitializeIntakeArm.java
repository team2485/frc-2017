package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.ZeroGearIntakeEncoder;
import org.usfirst.frc.team2485.subsystems.LowerGearIntakeArm;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * @author Ben Dorsey
 */

public class InitializeIntakeArm extends CommandGroup {
	public InitializeIntakeArm() {
		addSequential(new LowerGearIntakeArm(2000));
		addSequential(new ZeroGearIntakeEncoder());
	}
}
