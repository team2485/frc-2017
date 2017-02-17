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

			String report;

			if (driftRate == 0) {
				report = "WARNING:DRIFT RATE IS EXACTLY 0";
			} else if (Math.abs(driftRate) < 0.1) {
				report = "OK";
			} else {
				report = "FAILED:[" + driftRate + "]";
			}

			table.putString("AHRSDrift", report);

			done = true;
		}
	}

	@Override
	protected boolean isFinished() {
		return done;
	}
}
