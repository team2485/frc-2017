package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class GearHolder extends Subsystem {

	public void setOpen(boolean open) {

		RobotMap.gearSolenoid1.set(open);
		RobotMap.gearSolenoid2.set(!open);
	}

	@Override
	protected void initDefaultCommand() {
	}
}
