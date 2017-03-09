package org.usfirst.frc.team2485.robot.commands.selftest;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * @author Nicholas Contreras
 */

public class TestShooterSpeed extends Command {

	private long startTime;
	private boolean done, failed;
	private int correctSpeedCount;
	private ITable table;

	@Override
	protected void initialize() {
		startTime = System.currentTimeMillis();
		failed = false;
		RobotMap.shooter.setTargetSpeed(ConstantsIO.kShotRPS);
		table = NetworkTable.getTable("SmartDashboard").getSubTable("SelfTest");
	}

	@Override
	protected void execute() {

		double curRate = RobotMap.shooter.getAvgError();

		if (System.currentTimeMillis() - 5000 < startTime) {

			if (Math.abs(curRate) < 1) {
				correctSpeedCount++;
			} else {
				correctSpeedCount = 0;
			}

			if (correctSpeedCount > 50) {
				failed = false;
				done = true;
			}
		} else {
			failed = true;
			done = true;
		}
	}

	@Override
	protected boolean isFinished() {
		return done;
	}

	@Override
	protected void end() {

		RobotMap.shooter.disableShooter();

		int spinUpTime = (int) (System.currentTimeMillis() - startTime);

		String report;

		if (failed) {
			report = "FAILED:Could not spin up in time";
		} else {
			if (spinUpTime < 1000) {
				report = "OK";
			} else {
				report = "WARNING:Took " + spinUpTime + "ms to spin up";
			}
		}

		table.putString("ShooterSpeed", report);
	}
}
