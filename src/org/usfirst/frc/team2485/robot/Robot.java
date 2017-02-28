
package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	private AutoPath path;

	public void robotInit() {
		ConstantsIO.init();
		RobotMap.init();
		OI.init();
		path = new AutoPath(AutoPath.getPointsForFunction((double t) -> {
			return new Pair(0, 300 * t);
		}, 10000));

		RobotMap.updateConstants();

	}

	public void disabledInit() {
		Scheduler.getInstance().disable();
		Scheduler.getInstance().removeAll();
		RobotMap.driveTrain.reset();
		RobotMap.intakeArm.reset();
		RobotMap.intakeRollers.reset();
		RobotMap.compressor.stop();
		RobotMap.shooter.disableShooter();
		RobotMap.feeder.disableFeeder();
		RobotMap.wheelOfDeath.stop();
	}

	public void disabledPeriodic() {
		updateSmartDashboard();
	}

	public void autonomousInit() {
		Scheduler.getInstance().enable();
		ConstantsIO.init();
		RobotMap.updateConstants();

		RobotMap.ahrs.zeroYaw();

		// CommandGroup group = new CommandGroup();
		// group.addSequential(new DriveTo(path, 150));
		// group.addSequential(new ResetDriveTrain());
		// Scheduler.getInstance().add(group);
		
//		CameraServer.getInstance().startAutomaticCapture();

		RobotMap.driveTrain.zeroEncoders();
		RobotMap.driveTrain.updateConstants();
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
		RobotMap.wheelOfDeath.setLastSpeed(RobotMap.wheelOfDeath.getSpeed());
		System.out.println("Speed: " + RobotMap.wheelOfDeath.getSpeed());
		System.out.println("PWM: " + RobotMap.wheelOfDeath.getPWM());

		// RobotMap.driveTrain.rotateTo(45);
		// RobotMap.driveTrain.setLeftRightVelocity(40, 40);

	}

	public void teleopInit() {
		Scheduler.getInstance().enable();
		ConstantsIO.init();
		RobotMap.updateConstants();
		RobotMap.driveTrain.zeroEncoders();
		// Scheduler.getInstance().add(new HighLowCurrentTest(8, 4, 0, 0, 4000,
		// 0, 1));

	}

	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
		System.out.println("swod distance" + RobotMap.swodEncoder.getDistance());

	}

	public void testInit() {
		ConstantsIO.init();
		RobotMap.updateConstants();

		throw new RuntimeException("Don't enable test mode");
	}

	public void testPeriodic() {
		updateSmartDashboard();
	}

	public void updateSmartDashboard() {
		// SmartDashboard.putNumber("avg error",
		// RobotMap.driveTrain.getLeftVelocityAvgError());
		SmartDashboard.putNumber("Left Current", RobotMap.driveLeft2.getOutputCurrent());
		SmartDashboard.putNumber("Right Current", RobotMap.driveRight2.getOutputCurrent());
		SmartDashboard.putNumber("Left Velocity Error", RobotMap.driveTrain.getLeftVelocityPIDError());
		SmartDashboard.putBoolean("Test Boolean", System.currentTimeMillis() % 1000 > 500);
		SmartDashboard.putNumber("Test Number", System.currentTimeMillis() % 1000);
		SmartDashboard.putNumber("LeftVelocity", RobotMap.driveEncRateLeft.pidGet());
		SmartDashboard.putNumber("RightVelocity", RobotMap.driveEncRateRight.pidGet());
		SmartDashboard.putNumber("Dist", RobotMap.averageEncoderDistance.pidGet());
		SmartDashboard.putNumber("Angle", RobotMap.ahrs.getAngle());
//		SmartDashboard.putNumber("Spinning Wheel of Death Current", RobotMap.deathMotor.getOutputCurrent());
//		SmartDashboard.putNumber("Spinning Wheel of Death Voltage", RobotMap.deathMotor.getOutputVoltage());
		SmartDashboard.putNumber("Average Angular Velocity Error", RobotMap.driveTrain.getAngularVelocityError());
		SmartDashboard.putNumber("Shooter Error", RobotMap.shooter.getAvgError());
		SmartDashboard.putNumber("Shooter Distance", RobotMap.shooterEncoder.getDistance());
		SmartDashboard.putNumber("Uptake Speed", RobotMap.feederEncoder.getRate());
		SmartDashboard.putNumber("Spinning Wheel of Death Speed", RobotMap.swodEncoder.getRate());
//		SmartDashboard.putNumber("Spinning Wheel of Death PWM", RobotMap.wheelOfDeath.getPWM());

		NetworkTable.getTable("SmartDashboard").getSubTable("Temperatures").putNumber("Left Drive 1",
				RobotMap.driveLeft1.getTemperature());
		NetworkTable.getTable("SmartDashboard").getSubTable("Temperatures").putNumber("Left Drive 2",
				RobotMap.driveLeft2.getTemperature());
		NetworkTable.getTable("SmartDashboard").getSubTable("Temperatures").putNumber("Right Drive 1",
				RobotMap.driveRight1.getTemperature());
		NetworkTable.getTable("SmartDashboard").getSubTable("Temperatures").putNumber("Right Drive 2",
				RobotMap.driveRight2.getTemperature());
		NetworkTable.getTable("SmartDashboard").getSubTable("Temperatures").putNumber("Wheel of Death",
				RobotMap.deathMotor.getTemperature());

	}
}
