package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.InstantCommand;
/**
 * 
 * @author Vicky
 *
 */

public class RunWheelOfDeath extends InstantCommand {
	private boolean on; 
	
	public RunWheelOfDeath(boolean on){
		this.on = on;
	}
	
	@Override
	protected void initialize() {
		if (on) {
			RobotMap.wheelOfDeath.setPWM(.2);;
		} else {
			RobotMap.wheelOfDeath.setPWM(0);
		}
	}
	
}
