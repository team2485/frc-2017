package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ExtendIntakeArm extends InstantCommand{
	
	private Solenoid intakeArmSolenoid;
	private boolean booooo;
	
	public ExtendIntakeArm(boolean booooo){
		requires(RobotMap.intakeArm);
		this.booooo = booooo;
		
	}
	
	public void initialize(){
		if(booooo)
			RobotMap.intakeArm.extend();
		else
			RobotMap.intakeArm.reset();
	}

}
