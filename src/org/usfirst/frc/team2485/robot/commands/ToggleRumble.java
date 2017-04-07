package org.usfirst.frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * @author Ben Dorsey
 */

public class ToggleRumble extends InstantCommand {
	private double rumbleLeft, rumbleRight;
	private XboxController xbox;
	public ToggleRumble(XboxController xbox, double rumbleLeft, double rumbleRight) {
		this.xbox = xbox;
		this.rumbleLeft = rumbleLeft;
		this.rumbleRight = rumbleRight;
	}
	
	@Override
	protected void initialize() {
		super.initialize();
		xbox.setRumble(RumbleType.kLeftRumble, rumbleLeft);
		xbox.setRumble(RumbleType.kRightRumble, rumbleRight);
	}

}
