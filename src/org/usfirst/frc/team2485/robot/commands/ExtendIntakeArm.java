package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ExtendIntakeArm extends InstantCommand{
	
	private Solenoid intakeArmSolenoid;
	
	public ExtendIntakeArm(){
		requires(RobotMap.intakeArm);
		
	}
	
	public void initialize(){
		RobotMap.intakeArm.extend();
	}
		

}
