package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.ResetDriveTrain;
import org.usfirst.frc.team2485.robot.commands.SetGearWingsPosition;
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
			boolean isBoiler = isRed == (airshipSide == AirshipSide.RIGHT_SIDE);
			double offset = isBoiler ? 0 : 5;
			AutoPath path = (airshipSide == AirshipSide.LEFT_SIDE) ? 
					new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(offset, 0), new Pair(offset, 75), new Pair(25, 91), new Pair(72, 112))) 
					: new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(-offset,0), new Pair(-offset, 75), new Pair(-49, 92), new Pair(-71, 106)));
			addSequential(new DriveTo(path, 75, false, 4000));
			addSequential(new ResetDriveTrain());
			addSequential(new ZeroEncoders());
			addSequential(new SetGearWingsPosition(true));
			addSequential(new TimedCommand(.5));
			addSequential(new DriveStraight(-80, (airshipSide == AirshipSide.LEFT_SIDE) ? 60 : 300, 100, 3000));
			addSequential(new ResetDriveTrain());
			addSequential(new ZeroEncoders());
			path = new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(-2, 66), new Pair(-33, 85), new Pair(-28, 158.5), new Pair(-27, 211)));
			addSequential(new DriveTo(path, 100, false, 6000));
		}
	}
}
