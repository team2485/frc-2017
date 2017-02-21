package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.SetGearChutePosition;
import org.usfirst.frc.team2485.robot.commands.SetGearHolderPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ResetGear extends CommandGroup {
	public ResetGear() {
		addParallel(new SetGearChutePosition(false));
		addParallel(new SetGearHolderPosition(false));
	}
}
