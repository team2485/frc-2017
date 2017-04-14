package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.SetGearWingsPosition;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class DeadReckoningGearAuto extends CommandGroup {
	private static final double CONTROL_LENGTH_OUT = 75, CONTROL_LENGTH_IN = 28;
	public DeadReckoningGearAuto() {
		double x = RobotMap.autoDeadReckoning.getX();
		double y = RobotMap.autoDeadReckoning.getY();
		double curAngle = RobotMap.ahrs.getAngle();
		AutoPath placeGearPath = new AutoPath(AutoPath.getPointsForBezier(10000, new Pair(x, y), 
				new Pair(x + CONTROL_LENGTH_OUT * Math.sin(Math.toRadians(curAngle)), y + CONTROL_LENGTH_OUT * Math.cos(Math.toRadians(curAngle))),
				new Pair(0, -CONTROL_LENGTH_IN), new Pair(0, 0)));
		DriveTo drivePlaceGear = new DriveTo(placeGearPath, 75, false, 5000);
		addSequential(drivePlaceGear);
		addSequential(new SetGearWingsPosition(true));
		addSequential(new TimedCommand(0.5));
		addSequential(new DriveStraight(-50, 0, 75, 3000));
	}
}
