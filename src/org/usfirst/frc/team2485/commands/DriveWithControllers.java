package org.usfirst.frc.team2485.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithControllers extends Command{
	
	public DriveWithControllers(){
		requires(RobotMap.driveTrain);
	}
	
	@Override
	protected void execute() {
		double foward = -OI.xBox.getRawAxis(1);
		double right = OI.xBox.getRawAxis(4);
		RobotMap.driveTrain.warlordDrive(foward, right);
		
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
