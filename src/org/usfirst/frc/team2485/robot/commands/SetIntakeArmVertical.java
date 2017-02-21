package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetIntakeArmVertical extends InstantCommand {

	private boolean extended;

	public SetIntakeArmVertical(boolean extended) {
		requires(RobotMap.intakeArm);
		this.extended = extended;

	}

	public void initialize() {
		RobotMap.intakeArm.setVertical(extended);
	}

}
