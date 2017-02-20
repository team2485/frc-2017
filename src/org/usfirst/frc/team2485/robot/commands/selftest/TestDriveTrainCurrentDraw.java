package org.usfirst.frc.team2485.robot.commands.selftest;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

public class TestDriveTrainCurrentDraw extends Command {
	private double currentLeft1, currentLeft2, currentLeftMini, currentRight1, currentRight2, currentRightMini;
	private double startTime;
	private boolean isFinished;

	@Override
	protected void initialize() {
		RobotMap.driveTrainLeft.set(1);
		RobotMap.driveTrainRight.set(1);
		startTime = System.currentTimeMillis();
		isFinished = false;
	}

	@Override
	protected void execute() {
		if ((System.currentTimeMillis() - startTime) > 3000) {
			currentLeft1 = RobotMap.driveLeft1.getOutputCurrent();
			currentLeft2 = RobotMap.driveLeft2.getOutputCurrent();
			currentLeftMini = RobotMap.driveLeftMini.getOutputCurrent();
			currentRight1 = RobotMap.driveRight1.getOutputCurrent();
			currentRight2 = RobotMap.driveRight2.getOutputCurrent();
			currentRightMini = RobotMap.driveRightMini.getOutputCurrent();
			isFinished = true;
		}
	}

	@Override
	protected boolean isFinished() {
		return isFinished;
	}
	
	@Override
	protected void end() {
		RobotMap.driveTrainLeft.set(0);
		RobotMap.driveTrainRight.set(0);
		
		ITable table = NetworkTable.getTable("SmartDashboard").getSubTable("SelfTest");
		
		if (currentLeftMini < currentLeft1 && currentLeftMini < currentLeft2) {
			table.putString("LeftMiniCurrentTest", "OK");
		} else {
			if (currentLeft1 < currentLeft2) {
				table.putString("LeftMiniCurrentTest", "FAILED:Device ID " + RobotMap.driveLeftPortCIM1 + " is Mini CIM");
			} else {
				table.putString("LeftMiniCurrentTest", "FAILED:Device ID " + RobotMap.driveLeftPortCIM2 + " is Mini CIM");
			}
		}
		
		
		if (currentRightMini < currentRight1 && currentRightMini < currentRight2) {
			table.putString("RightMiniCurrentTest", "OK");
		} else {
			if (currentRight1 < currentRight2) {
				table.putString("RightMiniCurrentTest", "FAILED:Device ID " + RobotMap.driveRightPortCIM1 + " is Mini CIM");
			} else {
				table.putString("RightMiniCurrentTest", "FAILED:Device ID " + RobotMap.driveRightPortCIM2 + " is Mini CIM");
			}
		}
	}

}
