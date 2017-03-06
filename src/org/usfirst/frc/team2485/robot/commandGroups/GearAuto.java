package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.CheckDistError;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveStraightConditional;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.RotateTo;
import org.usfirst.frc.team2485.robot.commands.SetGearWingsPosition;
import org.usfirst.frc.team2485.robot.commands.ZeroEncoders;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * @author Ben Dorsey
 */

public class GearAuto extends CommandGroup {
	public enum AirshipSide {
		CENTER, LEFT_SIDE, RIGHT_SIDE
	}
	
	public GearAuto(AirshipSide airshipSide, boolean isRed) {
		if (airshipSide == AirshipSide.CENTER) {
			addSequential(new DriveStraight(80, 0, 50, 6000));
			addSequential(new ResetDriveTrain());
			addSequential(new ZeroEncoders());
			addSequential(new SetGearWingsPosition(true));
			addSequential(new TimedCommand(.5));
			addSequential(new DriveStraight(-24, 0, 50, 6000));
			addSequential(new ResetDriveTrain());
		} else {  
			boolean isRight = (airshipSide == AirshipSide.RIGHT_SIDE);
			boolean isBoiler = isRed == isRight;
			double offset = isBoiler ? 0 : 5;
			AutoPath path = isRight ? 
				 new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(-offset,0), new Pair(-offset, 75), new Pair(-49, 92), new Pair(-72, 107))):
				new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(offset, 0), new Pair(offset, 75), new Pair(51, 102), new Pair(70, 115)));
			addSequential(new DriveTo(path, 75, false, 4000));
			addSequential(new ResetDriveTrain());
			addSequential(new ZeroEncoders());
//			CommandGroup reUnUltraMegaHookGear = new CommandGroup();
			addSequential(new CheckDistError());
			addSequential(new DriveStraightConditional(-3, isRight ? 300 : 60, 100, 1000, 1));
			addSequential(new ResetDriveTrain());
			addSequential(new ZeroEncoders());
			addSequential(new DriveStraightConditional(16, isRight ? 310 : 50, 100, 1000, 1));
			addSequential(new ResetDriveTrain());
			addSequential(new ZeroEncoders());
//			ConditionalCommand checkGear = new ConditionalCommand(reUnUltraMegaHookGear) {
//				@Override
//				protected boolean condition() {
//					return RobotMap.driveTrain.getDistanceError() > 5;
//				}
//			};
//			addSequential(checkGear);
			addSequential(new SetGearWingsPosition(true));
			addSequential(new TimedCommand(.5));
			addSequential(new DriveStraight(-80, isRight ? 300 : 60, 100, 3000));
			addSequential(new ResetDriveTrain());
			addSequential(new ZeroEncoders());
			int sign = isRight ? -1 : 1;
			path = new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(sign * 2, 66), new Pair(sign * 33, 85), new Pair(sign * 28, 158.5), new Pair(sign * 27, 370)));
			addSequential(new DriveTo(path, 100, false, 6000));
		}
	}
}
