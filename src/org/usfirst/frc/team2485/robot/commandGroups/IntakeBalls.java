package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.SetIntakeArmHorizontal;
import org.usfirst.frc.team2485.robot.commands.SetRollers;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class IntakeBalls extends CommandGroup{
	
	public IntakeBalls(){
		addParallel(new SetIntakeArmHorizontal(true));
		addParallel(new SetRollers(true));
	}

}
