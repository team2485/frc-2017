package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.commands.RunRollers;
import org.usfirst.frc.team2485.robot.commands.ToggleRumble;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;

/**
 * @author Ben Dorsey
 */

public class IntakeGear extends CommandGroup {
	public IntakeGear() {
//		addSequential(new SetIntakeArm(GearIntakeArm.GROUND));
		addSequential(new ToggleRumble(OI.elliotRumble, 1, 1));
		addSequential(new RunRollers());
		addSequential(new ToggleRumble(OI.elliotRumble, 0, 0));
//		addSequential(new SetIntakeArm(GearIntakeArm.STOWED));
	}
	@Override
	protected void end() {
		super.end();
		Scheduler.getInstance().add(new Rumble(OI.benRumble, 1, 1, .25));
		OI.elliotRumble.setRumble(RumbleType.kLeftRumble, 0);
		OI.elliotRumble.setRumble(RumbleType.kRightRumble, 0);
	}
	
	@Override
	protected void interrupted() {
		super.interrupted();
		end();
	}
}
