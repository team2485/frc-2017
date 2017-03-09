package org.usfirst.frc.team2485.robot.commands.selftest;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * @author Nicholas Contreras
 */

public class TestShooterMotorDirections extends Command {

	private long startTime;
	private boolean done;

	private double speed1, speed2;

	private ITable table;

	@Override
	protected void initialize() {
		startTime = System.currentTimeMillis();
		done = false;
		table = NetworkTable.getTable("SmartDashboard").getSubTable("SelfTest");
	}

	@Override
	protected void execute() {

		int timeSinceStart = (int) (System.currentTimeMillis() - startTime);

		if (timeSinceStart < 1000) {
			RobotMap.shooterMotors.getController(0).set(0.5);

			if (timeSinceStart > 900) {
				speed1 = RobotMap.shooterEncoder.getRate();
				RobotMap.shooterMotors.getController(0).set(0);
			}
		} else if (timeSinceStart < 2000) {
			RobotMap.shooterMotors.getController(1).set(0.5);

			if (timeSinceStart > 900) {
				speed1 = RobotMap.shooterEncoder.getRate();
				RobotMap.shooterMotors.getController(1).set(0);
			}
		} else {
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

		String result;

		if (speed1 > 0 && speed2 > 0) {
			result = "OK";
		} else {
			result = "FAILED:[" + speed1 + "," + speed2 + "]";
		}

		table.putString("ShooterMotorDirections", result);
	}
}
