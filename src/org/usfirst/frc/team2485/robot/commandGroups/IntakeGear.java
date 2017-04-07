package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.commands.Rumble;
import org.usfirst.frc.team2485.robot.commands.RunRollers;
import org.usfirst.frc.team2485.robot.commands.ToggleRumble;
import org.usfirst.frc.team2485.subsystems.GearIntakeArm;
import org.usfirst.frc.team2485.subsystems.SetIntakeArm;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.CommandGroup;

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
		addSequential(new Rumble(OI.benRumble, 1, 1, .25));
	}
	@Override
	protected void end() {
		super.end();
		OI.elliotRumble.setRumble(RumbleType.kLeftRumble, 0);
		OI.elliotRumble.setRumble(RumbleType.kRightRumble, 0);
	}
	
	@Override
	protected void interrupted() {
		super.interrupted();
		end();
	}
}
