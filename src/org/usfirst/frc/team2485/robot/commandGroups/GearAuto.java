package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.CheckDistError;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.RotateTo;
import org.usfirst.frc.team2485.robot.commands.SetGearWingsPosition;
import org.usfirst.frc.team2485.robot.commands.SetSWODSpeed;
import org.usfirst.frc.team2485.robot.commands.SetShooter;
import org.usfirst.frc.team2485.robot.commands.ZeroDriveEncoders;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.ConditionalCommandGroup;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * @author Ben Dorsey
 */

public class GearAuto extends CommandGroup {
	public enum AirshipSide {
		CENTER, LEFT_SIDE, RIGHT_SIDE
	}
	

	public GearAuto(AirshipSide airshipSide, boolean isRed, boolean shoot) {

		// @formatter:off
		RobotMap.ahrs.zeroYaw();

		if (airshipSide == AirshipSide.CENTER) {
			int sign = isRed ? 1 : -1;
			DriveTo center = new DriveTo(new AutoPath(AutoPath.getPointsForBezier(4000, new Pair(0, 0), new Pair(0, 86))), 50, false, 4000);
			center.setTolerance(5);
			addSequential(center);
			addSequential(new CheckDistError());
			ConditionalCommandGroup correctionGroup = new ConditionalCommandGroup(() -> {
				return !RobotMap.driveTrain.getAutoError();
			});
			RotateTo correctionRotation = new RotateTo(isRed ? 10 : 350
					
					, 1000);
			correctionGroup.addSequential(correctionRotation);
			correctionGroup.addSequential(new ResetDriveTrain());
			correctionGroup.addSequential(new ZeroDriveEncoders());
			AutoPath correctionPath = new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(0, 0), new Pair(sign * 2, 5), new Pair(sign * 2, 15)));
			DriveTo correction = new DriveTo(correctionPath, 40, false, 4000);
			correctionGroup.addSequential(correction);
			correctionGroup.addSequential(new ResetDriveTrain());
			correctionGroup.addSequential(new ZeroDriveEncoders());
			addSequential(correctionGroup);
			addSequential(new SetGearWingsPosition(true));
			addSequential(new TimedCommand(.5));
			if (shoot) {
				AutoPath centerShoot = isRed ?
						new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(0, -20.5), new Pair(0, 0)), AutoPath.getPointsForBezier(10000, new Pair(41, -75), new Pair(131.5, 97.5), new Pair(0, -39.5), new Pair(0, -20.5))) :
						new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(-49.28, -21.1), new Pair(-113.5, -57), new Pair(0, -48.5), new Pair(0, -20.5)), AutoPath.getPointsForBezier(10000, new Pair(0, -20.5), new Pair(0, 0)));
				addSequential(new DriveTo(centerShoot, 40, true, 40000)); //-49.28, -19.93 21.1
				addSequential(new ResetDriveTrain());
				addSequential(new ZeroDriveEncoders());
				addSequential(new SetShooter(true));
				addSequential(new SetSWODSpeed());
			}
		} else {  
			boolean isRight = (airshipSide == AirshipSide.RIGHT_SIDE);
			boolean isBoiler = isRed == isRight;
			double offset = isBoiler ? 0 : 5;
			if (airshipSide == AirshipSide.LEFT_SIDE && !isBoiler) {
				offset = 7;
			}
			int sign = isRight ? -1 : 1;
			AutoPath path = isRight ? 
				 new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(-offset, 2), new Pair(-offset, 75), new Pair(-52.25, 94), new Pair(-75.25, 109))):
				new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(offset, 2), new Pair(offset, 75), new Pair(54.13, 105.23), new Pair(73.13, 117.23))); //46, 99, 65, 112
			DriveTo drivePath = new DriveTo(path, 75, false, 4000);
			drivePath.setTolerance(5);
			addSequential(drivePath);
			addSequential(new ResetDriveTrain());
			addSequential(new ZeroDriveEncoders());
			addSequential(new CheckDistError());
			ConditionalCommandGroup correctionGroup = new ConditionalCommandGroup(() -> {
				return !RobotMap.driveTrain.getAutoError();
			});
			RotateTo correctionRotation = new RotateTo(isRight ? 290 : 70, 1000);
			correctionGroup.addSequential(correctionRotation);
			correctionGroup.addSequential(new ResetDriveTrain());
			correctionGroup.addSequential(new ZeroDriveEncoders());
			AutoPath correctionPath = new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(0, 0), new Pair(sign * 5.33, 0.77), new Pair(sign * 13.99, 5.77)));
			DriveTo correction = new DriveTo(correctionPath, 40, false, 4000);
			correctionGroup.addSequential(correction);
			correctionGroup.addSequential(new ResetDriveTrain());
			correctionGroup.addSequential(new ZeroDriveEncoders());
			addSequential(correctionGroup);
			addSequential(new SetGearWingsPosition(true));
			addSequential(new TimedCommand(.5)); 
			
			if (shoot && isBoiler) {
				path = isRight ? 
						new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(-70, 43.5), 
								new Pair(-70 + 48 * Math.sin(Math.toRadians(54)), 43.5 - 48 * Math.cos(Math.toRadians(54))), 
								new Pair(0, 56), new Pair(-72.0, 107))) : 
						new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(57, 59.0), new Pair(57-48*Math.sin(Math.toRadians(65)), 59-48*Math.cos(Math.toRadians(65))), 
								new Pair(-4.5, 64.5), new Pair(70.0, 115.0)));
				CommandGroup drive = new CommandGroup();
				drive.addSequential(new DriveTo(path, 50, true, 10000));
				drive.addSequential(new ResetDriveTrain());
				drive.addSequential(new ZeroDriveEncoders());
				drive.addSequential(new RotateTo(180 + (isRight ? -55 : 65), 4000)); //weird values we added at 2 am, so if they don't work fix them
				drive.addSequential(new ResetDriveTrain());
				drive.addSequential(new ZeroDriveEncoders());
				drive.addSequential(new SetShooter(true));
				drive.addSequential(new SetSWODSpeed());
				addParallel(drive);
			} else {
				addSequential(new DriveStraight(-80, isRight ? 305 : 55, 100, 3000));
				addSequential(new ResetDriveTrain());
				addSequential(new ZeroDriveEncoders());
				path = new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(sign * 2, 66), new Pair(sign * 33, 85), new Pair(sign * 28, 158.5), new Pair(sign * 27, 370)));
				addSequential(new DriveTo(path, 100, false, 6000));
			}
		}
		// @formatter:on

	}
	
}