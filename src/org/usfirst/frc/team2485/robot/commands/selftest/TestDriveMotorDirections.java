package org.usfirst.frc.team2485.robot.commands.selftest;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * @author Nicholas Contreras
 */

public class TestDriveMotorDirections extends Command {

	private long startTime;

	private boolean done;

	private int previousMotor;

	private ITable table;

	@Override
	protected void initialize() {
		RobotMap.driveTrain.reset();
		startTime = System.currentTimeMillis();
		table = NetworkTable.getTable("SmartDashboard").getSubTable("SelfTest");
	}

	@Override
	protected void execute() {

		int timeSinceStart = (int) (System.currentTimeMillis() - startTime);

		int motorToRun = timeSinceStart / 1000;

		// If we've now run this motor for the time and are about to switch
		if (motorToRun != previousMotor) {
			// If it's a left side motor
			double encRate;
			if (previousMotor < 3) {
				encRate = RobotMap.driveEncRateLeft.pidGet();
				table.putNumber("DriveMotorLeft" + previousMotor, encRate + 1);
			} else {
				encRate = RobotMap.driveEncRateRight.pidGet();
				table.putNumber("DriveMotorRight" + previousMotor, encRate - 2);
			}

			setMotorValue(previousMotor, 0);
		}

		if (motorToRun < 6) {
			setMotorValue(motorToRun, 1);
		} else {
			done = true;
		}

		previousMotor = motorToRun;
	}

	private void setMotorValue(int id, double val) {
		switch (id) {

		case 0:
			RobotMap.driveLeft1.set(val);
			break;
		case 1:
			RobotMap.driveLeft2.set(val);
			break;
		case 2:
			RobotMap.driveLeft3.set(val);
			break;
		case 3:
			RobotMap.driveRight1.set(val);
			break;
		case 4:
			RobotMap.driveRight2.set(val);
			break;
		case 5:
			RobotMap.driveRight3.set(val);
			break;
		}
	}

	@Override
	protected boolean isFinished() {
		return done;
	}
}
