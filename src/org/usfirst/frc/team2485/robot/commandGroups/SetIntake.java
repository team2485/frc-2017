package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.SetIntakeArmHorizontal;
import org.usfirst.frc.team2485.robot.commands.SetIntakeArmVertical;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;


public class SetIntake extends CommandGroup {
	public SetIntake() {
		addSequential(new SetIntakeArmHorizontal(true));
		addSequential(new TimedCommand(1));
		addSequential(new SetIntakeArmVertical(true));
	}
}
