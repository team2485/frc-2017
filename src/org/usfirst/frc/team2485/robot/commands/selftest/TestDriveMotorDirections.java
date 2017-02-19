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

	private double forwardVal, reverseVal;

	private boolean done;

	private int previousMotor;

	private ITable table;
	
	public TestDriveMotorDirections() {
		requires(RobotMap.driveTrain);
	}

	@Override
	protected void initialize() {
		RobotMap.driveTrain.reset();
		startTime = System.currentTimeMillis();
		table = NetworkTable.getTable("SmartDashboard").getSubTable("SelfTest");
		done = false;
	}

	@Override
	protected void execute() {

		int timeSinceStart = (int) (System.currentTimeMillis() - startTime);

		int motorToRun = timeSinceStart / 2000;

		int cycleTime = timeSinceStart % 2000;

		if (motorToRun != previousMotor) {
			publishTestResult(previousMotor);

			setMotorValue(previousMotor, 0);
		}

		if (motorToRun < 6) {
			testMotor(motorToRun, cycleTime);
		} else {
			done = true;
		}

		previousMotor = motorToRun;
	}

	private void publishTestResult(int motorID) {
		String report;

		if (forwardVal > 0.5 && reverseVal < -0.5) {
			report = "OK";
		} else {
			report = "FAILED:[" + forwardVal + ", " + reverseVal + "]";
		}

		if (motorID < 3) {
			table.putString("DriveMotorLeft" + (motorID + 1), report);
		} else {
			table.putString("DriveMotorRight" + (motorID - 2), report);
		}
	}

	private void testMotor(int motorToRun, int cycleTime) {
		double motorValue = 1;

		if (cycleTime > 1000) {
			motorValue = -1;
		}

		setMotorValue(motorToRun, motorValue);

		if (cycleTime > 900 && cycleTime < 1000) {
			forwardVal = getRateForMotor(motorToRun);
		}

		if (cycleTime > 1900) {
			reverseVal = getRateForMotor(motorToRun);
		}
	}

	private double getRateForMotor(int motorID) {
		if (motorID < 3) {
			return RobotMap.driveEncRateLeft.pidGet();
		} else {
			return RobotMap.driveEncRateRight.pidGet();
		}
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
			RobotMap.driveLeftMini.set(val);
			break;
		case 3:
			RobotMap.driveRight1.set(val);
			break;
		case 4:
			RobotMap.driveRight2.set(val);
			break;
		case 5:
			RobotMap.driveRightMini.set(val);
			break;
		}
	}

	@Override
	protected boolean isFinished() {
		return done;
	}
}
