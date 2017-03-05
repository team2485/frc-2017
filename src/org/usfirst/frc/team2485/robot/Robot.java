
package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.SetGearWingsPosition;
import org.usfirst.frc.team2485.robot.commands.SetLeftRightVelocity;
import org.usfirst.frc.team2485.robot.commands.ZeroEncoders;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;
import org.usfirst.frc.team2485.util.CommandTimeout;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	private AutoPath path1, path2, path3, pathTest4;

	public void robotInit() {
		ConstantsIO.init();
		RobotMap.init();
		OI.init();
		path1 = new AutoPath(AutoPath.getPointsForFunction((double t) -> {
			return new Pair(0, 84*t);
		}, 10000));
		path2 = new AutoPath(AutoPath.getPointsForFunction((double t) -> {
			return new Pair (0, 24*t);
		}, 10000));
		path3  = new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(0, 0), 
				new Pair(0, 75), new Pair(25, 91), new Pair(72, 112)));
		pathTest4 = new AutoPath(AutoPath.getPointsForFunction((double t) -> {
			return new Pair (0, 12*t);
		}, 10000));
//		path = AutoPath.getPointsForBezier(10000, new Pair(0, 0), new Pair(x, y))
//		path = new AutoPath(AutoPath.getPointsForFunction((double t) -> {
//			return new Pair(50*(1-Math.cos(Math.PI*t)), 50*Math.sin(Math.PI*t));
//		}, 10000));

		RobotMap.updateConstants();

	}

	public void disabledInit() {
		Scheduler.getInstance().disable();
		Scheduler.getInstance().removeAll();
		RobotMap.driveTrain.reset();
		RobotMap.gearHolder.reset();
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
		
		RobotMap.driveTrain.zeroEncoders();
		RobotMap.driveTrain.updateConstants();

//		 CommandGroup group = new CommandGroup();
//		 group.addSequential(new DriveStraight(80, 0, 50, 6000));
//		 group.addSequential(new ResetDriveTrain());
//		 group.addSequential(new ZeroEncoders());
//		 group.addSequential(new SetGearWingsPosition(true));
//		 group.addSequential(new TimedCommand(.5));
//		 group.addSequential(new DriveStraight(-24, 0, 50, 6000));
//		 group.addSequential(new ResetDriveTrain());
//		 Scheduler.getInstance().add(group);
		 CommandGroup group = new CommandGroup();
		 group.addSequential(new DriveTo(path3, 25, false, 8000));
		 group.addSequential(new ResetDriveTrain());
		 group.addSequential(new ZeroEncoders());
		 group.addSequential(new SetGearWingsPosition(true));
		 group.addSequential(new TimedCommand(.5));
		 group.addSequential(new DriveStraight(-24, 60, 25, 6000));
		 Scheduler.getInstance().add(group);
//		
//		CommandGroup group = new CommandGroup();
//		group.addSequential(new SetLeftRightVelocity(40, 40));
//		Scheduler.getInstance().add(group);
		
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
		RobotMap.wheelOfDeath.setLastSpeed(RobotMap.wheelOfDeath.getSpeed());

//		 RobotMap.driveTrain.rotateTo(45);
//		 RobotMap.driveTrain.setLeftRightVelocity(40, 40);

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
		SmartDashboard.putNumber("Spinning Wheel of Death Current", RobotMap.deathMotor.getOutputCurrent());
//		SmartDashboard.putNumber("Spinning Wheel of Death Voltage", RobotMap.deathMotor.getOutputVoltage());
		SmartDashboard.putNumber("Average Angle Error", RobotMap.driveTrain.getAnglePIDError());
		SmartDashboard.putNumber("Average Angular Velocity Error", RobotMap.driveTrain.getAngularVelocityError());
		SmartDashboard.putNumber("Shooter Error", RobotMap.shooter.getAvgError());
		SmartDashboard.putNumber("Shooter Distance", RobotMap.shooterEncoder.getDistance());
		SmartDashboard.putNumber("Uptake Speed", RobotMap.feederEncoder.getRate());
		SmartDashboard.putNumber("Spinning Wheel of Death Speed", RobotMap.brokenSWODEnc.pidGet());
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
