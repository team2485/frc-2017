package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ExtendIntakeArm extends InstantCommand {

	private boolean extended;

	public ExtendIntakeArm(boolean extended) {
		requires(RobotMap.intakeArm);
		this.extended = extended;

	}

	public void initialize() {
		if (extended) {
			RobotMap.intakeArm.extend();
		} else {
			RobotMap.intakeArm.reset();
		}
	}

}
