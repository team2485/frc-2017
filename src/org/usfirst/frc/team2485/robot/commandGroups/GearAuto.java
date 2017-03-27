package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.CheckDistError;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveStraightConditional;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.RotateTo;
import org.usfirst.frc.team2485.robot.commands.SetGearWingsPosition;
import org.usfirst.frc.team2485.robot.commands.SetSWODSpeed;
import org.usfirst.frc.team2485.robot.commands.SetShooter;
import org.usfirst.frc.team2485.robot.commands.WaitTillClose;
import org.usfirst.frc.team2485.robot.commands.ZeroEncoders;
import org.usfirst.frc.team2485.util.AutoPath;
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
		
		if (airshipSide == AirshipSide.CENTER) {
			addSequential(new DriveStraight(80, 0, 50, 6000));
			addSequential(new ResetDriveTrain());
			addSequential(new ZeroEncoders());
			addSequential(new SetGearWingsPosition(true));
			addSequential(new TimedCommand(.5));
			addSequential(new DriveStraight(-24, 0, 50, 6000));
			addSequential(new ResetDriveTrain());
			if (shoot) {
				
			}
		} else {  
			boolean isRight = (airshipSide == AirshipSide.RIGHT_SIDE);
			boolean isBoiler = isRed == isRight;
			double offset = isBoiler ? 0 : 5;
			AutoPath path = isRight ? 
				 new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(-offset,0), new Pair(-offset, 75), new Pair(-52.25, 94), new Pair(-75.25, 109))):
				new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(offset, 0), new Pair(offset, 75), new Pair(48.63, 102.1), new Pair(67.63, 115.1))); //46, 99, 65, 112
			addSequential(new DriveTo(path, 75, false, 4000));
			addSequential(new ResetDriveTrain());
			addSequential(new ZeroEncoders());
//			addSequential(new CheckDistError());
//			addSequential(new DriveStraightConditional(-17, isRight ? 300 : 60, 100, 2000));
//			addSequential(new ResetDriveTrain());
//			addSequential(new ZeroEncoders());
//			addSequential(new DriveStraightConditional(27, isRight ? 305 : 55, 100, 2500));
//			addSequential(new ResetDriveTrain());
//			addSequential(new ZeroEncoders()); 
			addSequential(new SetGearWingsPosition(true));
			addSequential(new TimedCommand(.5)); 
			
			if (shoot && isBoiler) {
				path = isRight ? 
						new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(-78, 48.5), new Pair(-38, 19.5), new Pair(0, 56), new Pair(-72.0, 107))) : 
						new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(58.55, 60.0), new Pair(58.55-48*Math.sin(Math.toRadians(65)), 60-48*Math.cos(Math.toRadians(65))), new Pair(-4.5, 64.5), new Pair(70.0, 115.0)));
				CommandGroup drive = new CommandGroup();
				drive.addSequential(new DriveTo(path, 50, true, 10000));
				drive.addSequential(new ResetDriveTrain());
				drive.addSequential(new ZeroEncoders());
				drive.addSequential(new RotateTo(65+180, 4000));
				drive.addSequential(new ResetDriveTrain());
				drive.addSequential(new ZeroEncoders());
				drive.addSequential(new SetShooter(true));
				drive.addSequential(new SetSWODSpeed());
				addParallel(drive);
			} else {
				addSequential(new DriveStraight(-80, isRight ? 300 : 60, 100, 3000));
				addSequential(new ResetDriveTrain());
				addSequential(new ZeroEncoders());
				int sign = isRight ? -1 : 1;
				path = new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(sign * 2, 66), new Pair(sign * 33, 85), new Pair(sign * 28, 158.5), new Pair(sign * 27, 370)));
				addSequential(new DriveTo(path, 100, false, 6000));
			}
		}
		// @formatter:on

	}
	
}