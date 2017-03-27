package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.RunRollers;
import org.usfirst.frc.team2485.subsystems.GearIntakeArm;
import org.usfirst.frc.team2485.subsystems.SetIntakeArm;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * @author Ben Dorsey
 */

public class IntakeGear extends CommandGroup {
	public IntakeGear() {
		addSequential(new SetIntakeArm(GearIntakeArm.GROUND));
		addSequential(new RunRollers());
		addSequential(new SetIntakeArm(GearIntakeArm.STOWED));
	}
}
