
package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.robot.commandGroups.GearAuto;
import org.usfirst.frc.team2485.robot.commandGroups.GearAuto.AirshipSide;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.RotateTo;
import org.usfirst.frc.team2485.robot.commands.SetGearWingsPosition;
import org.usfirst.frc.team2485.robot.commands.SetLeftRightVelocity;
import org.usfirst.frc.team2485.robot.commands.ZeroEncoders;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;
import org.usfirst.frc.team2485.util.ConstantsIO;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {


	public void robotInit() {
		ConstantsIO.init();
		RobotMap.init();
		OI.init();
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
		
//		Tuning
//		CommandGroup group = new CommandGroup();
////		group.addSequential(new SetLeftRightVelocity(60, 60));
//		group.addSequential(new DriveStraight(200, 0, 100, 20000));
//		Scheduler.getInstance().add(group);
		
//		 DRIVERS IF YOU NEED TO CHANGE AUTO DO IT HERE

		Scheduler.getInstance().add(new GearAuto(AirshipSide.LEFT_SIDE, // which hook we score on, left, right, or center
				false, // true if we are red
				true)); // true if we should shoot, only set to true near boiler
		
	}

	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
	}

	public void teleopInit() {
		Scheduler.getInstance().enable();
		ConstantsIO.init();
		RobotMap.updateConstants();
		RobotMap.driveTrain.zeroEncoders();
	}

	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
//		System.out.println("pressure" + RobotMap.compressor.getPressureSwitchValue());
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
//		SmartDashboard.putNumber("Left Current", RobotMap.driveLeft2.getOutputCurrent());
//		SmartDashboard.putNumber("Right Current", RobotMap.driveRight2.getOutputCurrent());
		SmartDashboard.putNumber("Left Velocity Error", RobotMap.driveTrain.getLeftVelocityPIDError());
//		SmartDashboard.putBoolean("Test Boolean", System.currentTimeMillis() % 1000 > 500);
		SmartDashboard.putNumber("Test Number", System.currentTimeMillis() % 1000);
		SmartDashboard.putNumber("LeftVelocity", RobotMap.driveEncRateLeft.pidGet());
		SmartDashboard.putNumber("RightVelocity", RobotMap.driveEncRateRight.pidGet());
//		SmartDashboard.putNumber("Dist", RobotMap.averageEncoderDistance.pidGet());
//		SmartDashboard.putNumber("Angle", RobotMap.ahrs.getAngle());
		SmartDashboard.putNumber("Spinning Wheel of Death Current", RobotMap.deathMotor.getOutputCurrent());
		SmartDashboard.putNumber("Average Angle Error", RobotMap.driveTrain.getAnglePIDError());
		SmartDashboard.putNumber("Average Angular Velocity Error", RobotMap.driveTrain.getAngularVelocityError());
		SmartDashboard.putNumber("Shooter Error", RobotMap.shooter.getAvgError());
		SmartDashboard.putNumber("Uptake Speed", RobotMap.feederEncoder.getRate());
//		SmartDashboard.putNumber("Uptake Speed Error", RobotMap.feeder.getAvgError());
//		SmartDashboard.putNumber("SWOD Current", RobotMap.wheelOfDeath.getCurrent());
		SmartDashboard.putNumber("Distance Error", RobotMap.driveTrain.getDistanceError());
		SmartDashboard.putNumber("Distance", RobotMap.driveEncLeft.getDistance());
		SmartDashboard.putNumber("Left Vel Setpoint", RobotMap.driveTrain.velocityPIDLeft.getSetpoint());
		SmartDashboard.putNumber("Gyro Angle", RobotMap.ahrs.getAngle());


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
