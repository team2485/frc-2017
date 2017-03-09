package org.usfirst.frc.team2485.robot.commands.selftest;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.SetSWODSpeed;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * @author Nicholas Contreras
 */

public class TestSWODCurrentDraw extends Command {

	private long startTime;
	private boolean done;
	private ITable table;

	public TestSWODCurrentDraw() {
		requires(RobotMap.wheelOfDeath);
	}

	@Override
	protected void initialize() {
		startTime = System.currentTimeMillis();
		done = false;
		table = NetworkTable.getTable("SmartDashboard").getSubTable("SelfTest");
		RobotMap.wheelOfDeath.setPWM(SetSWODSpeed.NORMAL_POWER);
	}

	@Override
	protected void execute() {
		if (System.currentTimeMillis() - startTime > 500) {

			double val = RobotMap.wheelOfDeath.getCurrent();

			String result;

			if (val == 0) {
				result = "FAILED:Current draw is 0";
			} else if (val >= SetSWODSpeed.JAM_CURRENT_THRESHOLD) {
				result = "FAILED:Current draw above jam threshold";
			} else {
				result = "OK";
			}

			table.putString("SWODCurrentTest", result);

			RobotMap.wheelOfDeath.stop();
			done = true;
		}
	}

	@Override
	protected boolean isFinished() {
		return done;
	}
}
