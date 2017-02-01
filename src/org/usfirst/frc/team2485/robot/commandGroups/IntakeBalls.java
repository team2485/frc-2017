package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.ExtendIntakeArm;
import org.usfirst.frc.team2485.robot.commands.RollersOn;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class IntakeBalls extends CommandGroup{
	
	public IntakeBalls(){
		addParallel(new ExtendIntakeArm(true));
		addParallel(new RollersOn());
	}

}
