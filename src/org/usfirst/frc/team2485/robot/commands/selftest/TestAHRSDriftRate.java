package org.usfirst.frc.team2485.robot.commands.selftest;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * @author Nicholas Contreras
 */

public class TestAHRSDriftRate extends Command {

	private double startAngle;
	private boolean done;

	@Override
	protected void initialize() {
		startAngle = RobotMap.ahrs.getAngle();
	}

	@Override
	protected void execute() {
		if (timeSinceInitialized() > 1) {
			
			double driftRate = RobotMap.ahrs.getAngle() - startAngle;
			
			ITable table = NetworkTable.getTable("SmartDashboard").getSubTable("SelfTest");
			
			table.putNumber("AHRSDrift", driftRate);
			
			done = true;
		}
	}

	@Override
	protected boolean isFinished() {
		return done;
	}
}
