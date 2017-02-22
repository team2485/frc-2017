package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class TestVerticalSolenoid1 extends InstantCommand {
	private boolean extended;

	public TestVerticalSolenoid1(boolean extended) {
		requires(RobotMap.intakeArm);
		this.extended = extended;

	}

	public void initialize() {
		RobotMap.intakeArmSolenoidVertical1.set(extended);
	}


}
