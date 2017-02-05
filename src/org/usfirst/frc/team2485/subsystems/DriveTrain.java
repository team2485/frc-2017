package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.ScalingMax;
import org.usfirst.frc.team2485.util.ThresholdHandler;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @author Nicholas Contreras
 */

public class DriveTrain extends Subsystem {
	public enum DriveSpeed {
		SLOW_SPEED_RATING, NORMAL_SPEED_RATING, FAST_SPEED_RATING;

		public double getSpeedFactor() {

			switch (this) {
			case SLOW_SPEED_RATING:
				return 0.6;
			case NORMAL_SPEED_RATING:
				return 0.8;
			default:
				return 1.0;
			}
		}
	}

	private static final double STEERING_DEADBAND = 0.1;
	private static final double THROTTLE_DEADBAND = 0.2;
	private double driveSpeed = DriveSpeed.NORMAL_SPEED_RATING.getSpeedFactor();
	private static final double MAX_CURRENT = 1 / 0.07, MIN_CURRENT = 0;

	// private static final int MINIMUM_DRIVETO_ON_TARGET_ITERATIONS = 10;
	// private static final double ABS_TOLERANCE_DRIVETO_ANGLE = 0;
	// private static final double ABS_TOLERANCE_DRIVETO_DISTANCE = 0;
	// private static final double LOW_ENC_RATE = 0;
	// private static final int MINIMUM_AHRS_ON_TARGET_ITERATIONS = 0;
	// private int driveToOnTargetIterations;

	private boolean isQuickTurn;
	private boolean isCurrentLeft, isCurrentRight;
	private double oldWheel, quickStopAccumulator;
	private boolean useCurrent;
	private static final double SENSITIVITY_HIGH = 0.85, SENSITIVITY_LOW = 0.75;

	// private WarlordsPIDController driveToPID, rotateToPID;
	private WarlordsPIDController ratePIDRight, ratePIDLeft;

	private TransferNode throttleTransferNode;
	private WarlordsPIDController steeringPidController;
	// private int ahrsOnTargetCounter;
	private TransferNode steeringTransferNode;
	private PIDSource curvatureSource;
	private ScalingMax powerScalingMax;
	private RampRate throttleRamp;
	private PIDOutput motorModeSwitcherLeft, motorModeSwitcherRight;
	// excuse the variable name, this represents the thing that switches between
	// current and voltage mode depending on the power value
	private PIDSource prescaledPowerRight, prescaledPowerLeft;

	private static final double MIN_SPEED = 10;
	public DriveTrain() {

		motorModeSwitcherLeft = (double out) -> {
			if (Math.abs(out * MAX_CURRENT) >= MIN_CURRENT && useCurrent) {
				setCurrentModeLeft(true);
				RobotMap.driveTrainLeft.set(out * MAX_CURRENT);
			} else {
				setCurrentModeLeft(false);
				System.out.println(out);
				RobotMap.driveTrainLeft.set(out);
			}
		};

		motorModeSwitcherRight = (double out) -> {
			if (Math.abs(out * MAX_CURRENT) > MIN_CURRENT && useCurrent) {
				setCurrentModeRight(true);
				RobotMap.driveTrainRight.set(out * MAX_CURRENT);
			} else {
				setCurrentModeRight(false);
				RobotMap.driveTrainRight.set(out);
			}
		};

		throttleTransferNode = new TransferNode(0);
		steeringTransferNode = new TransferNode(0);

		throttleRamp = new RampRate(new PIDOutput[] { throttleTransferNode }, ConstantsIO.kUpRamp_Drive,
				ConstantsIO.kDownRamp_Drive);

		curvatureSource = new PIDSourceWrapper(() -> {
			double leftVelocity = RobotMap.driveEncLeft.getRate();
			double rightVelocity = RobotMap.driveEncRight.getRate();
			if (Math.abs(leftVelocity + rightVelocity) / 2 < MIN_SPEED) {
				steeringPidController.disable();
				return 0;
			} else {
				return (leftVelocity - rightVelocity) / (leftVelocity + rightVelocity);
			}
		});

		// now both from -1 -> 1 instead of in amps
		prescaledPowerRight = new PIDSourceWrapper(() -> {
			if (isQuickTurn) {
				return -steeringTransferNode.pidGet();
			} else {
				return throttleTransferNode.getOutput() * (1 - steeringTransferNode.getOutput());
			}
		});

		prescaledPowerLeft = new PIDSourceWrapper(() -> {
			if (isQuickTurn) {
				return steeringTransferNode.pidGet();
			} else {
				return throttleTransferNode.getOutput() * (1 + steeringTransferNode.getOutput());
			}
		});

		powerScalingMax = new ScalingMax(new PIDOutput[] { motorModeSwitcherLeft, motorModeSwitcherRight },
				new PIDSource[] { prescaledPowerLeft, prescaledPowerRight });
		powerScalingMax.setSetpoint(1);

		steeringPidController = new WarlordsPIDController(curvatureSource, steeringTransferNode);
		steeringPidController.setPID(ConstantsIO.kP_DriveSteering, ConstantsIO.kI_DriveSteering,
				ConstantsIO.kD_DriveSteering, ConstantsIO.kF_DriveSteering);

		ratePIDLeft = new WarlordsPIDController(RobotMap.driveEncRateLeft, motorModeSwitcherLeft);
		ratePIDRight = new WarlordsPIDController(RobotMap.driveEncRateRight, motorModeSwitcherRight);
		ratePIDLeft.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity,
				ConstantsIO.kF_DriveVelocity);
		ratePIDRight.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity,
				ConstantsIO.kF_DriveVelocity);

	}

	public void setDriveSpeed(DriveSpeed speed) {
		driveSpeed = speed.getSpeedFactor();
	}

	public void setQuickTurn(boolean quickTurn) {
		this.isQuickTurn = quickTurn;
	}

	/**
	 * W.A.R. Lord Drive This drive method is based off of Team 254's Ultimate
	 * Ascent cheesyDrive code.
	 *
	 * @param controllerY
	 *            controllerY should be positive for forward motion
	 * @param controllerX
	 */
	public void warlordDrive(double controllerY, double controllerX, boolean useCurrent, boolean hasCheese,
			boolean hasSteeringCorrection, boolean useVelocity) {

		double steering = ThresholdHandler.deadbandAndScale(controllerX, STEERING_DEADBAND, 0.01, 1);
		double throttle = ThresholdHandler.deadbandAndScale(controllerY, THROTTLE_DEADBAND, 0.01, 1);
		if (useVelocity) {

		} else {
			this.useCurrent = useCurrent;
			throttleRamp.enable();
			throttleRamp.setSetpoint(throttle);
			powerScalingMax.enable();
			double averageSpeed = (RobotMap.driveEncLeft.getRate() + RobotMap.driveEncRight.getRate()) / 2;
			if (Math.abs(averageSpeed) > MIN_SPEED && !isQuickTurn && hasSteeringCorrection) {
				steeringPidController.enable();
				steeringPidController.setSetpoint(steering);
			} else {
				steeringPidController.disable();
				steeringTransferNode.setOutput(steering);
			}
		}

	}
	// /**
	// * Used to drive in a curve using closed loop control, set startAngle =
	// * endAngle to drive straight <br>
	// * Uses cascaded PIDControllers to achieve optimal performance
	// *
	// * @param inches
	// * distance to drive (inside tread if curving)
	// * @param startAngle
	// * heading at beginning of turn
	// * @param endAngle
	// * desired heading at end of turn
	// * @param maxSpeed
	// * maximum speed of center of robot in inches / second
	// * @return true if target has been reached
	// */
	// public boolean driveToAndRotateTo(double inches, double startAngle,
	// double endAngle, double maxSpeed) {
	//
	// if (!driveToPID.isEnabled()) {
	// driveToPID.enable();
	// rotateToPID.enable();
	// rotateToPID.setSetpoint(startAngle);
	// }
	// driveToPID.setSetpoint(inches);
	//
	// driveToPID.setOutputRange(-maxSpeed, maxSpeed);
	// rotateToPID.setOutputRange(-maxSpeed, maxSpeed);
	//
	// // uses % of distance to calculate where to turn to
	// double percentDone = (RobotMap.driveEncLeft.getDistance() +
	// RobotMap.driveEncRight
	// .getDistance()) / 2 / (inches != 0 ? inches : 0.00000001);// don't
	// // divide
	// // by
	// // 0
	// if (percentDone > 1) {
	// percentDone = 1;
	// } else if (percentDone < 0) {
	// percentDone = 0;
	// }
	// rotateToPID.setSetpoint(startAngle + (endAngle - startAngle)
	// * percentDone);
	//
	// double encoderOutput = dummyDriveToEncoderOutput.get();
	// double rotateToOutput = dummyRotateToOutput.get();
	//
	// // use output from PIDControllers to calculate target velocities
	// double leftVelocity = encoderOutput + rotateToOutput;
	// double rightVelocity = encoderOutput - rotateToOutput;
	//
	// // ramp output from PIDControllers to prevent saturating velocity
	// // control loop
	// leftVelocity = leftVelocityRamp.getNextValue(leftVelocity);
	// rightVelocity = rightVelocityRamp.getNextValue(rightVelocity);
	//
	// setLeftRightVelocity(leftVelocity, rightVelocity);
	//
	// if (Math.abs(rotateToPID.getError()) < ABS_TOLERANCE_DRIVETO_ANGLE) {
	// ahrsOnTargetCounter++;
	// } else {
	// ahrsOnTargetCounter = 0;
	// }
	//
	// double avgVelocity = (RobotMap.driveEncLeft.getRate() +
	// RobotMap.driveEncRight
	// .getRate()) / 2;
	//
	// if (Math.abs(driveToPID.getError()) < ABS_TOLERANCE_DRIVETO_DISTANCE
	// && Math.abs(avgVelocity) < LOW_ENC_RATE
	// && ahrsOnTargetCounter >= MINIMUM_AHRS_ON_TARGET_ITERATIONS) {
	//
	// setLeftRightVelocity(0.0, 0.0); // actively stops driveTrain
	// driveToPID.disable();
	// rotateToPID.disable();
	// return true;
	//
	// }
	//
	// return false;
	//
	// }

	// /**
	// * Sends outputs values to the left and right side of the drive base after
	// * scaling based on virtual gear. <br>
	// * The parameters should both be positive to move forward. One side has
	// * inverted motors...do not send a negative to one side and a positive to
	// * the other for forward or backwards motion.
	// *
	// * @param leftOutput
	// * @param rightOutput
	// */
	// public void setLeftRight(double leftOutput, double rightOutput) {
	//
	//// driveToPID.disable();
	//
	// ratePIDLeft.disable();
	// ratePIDRight.disable();
	//
	// RobotMap.driveTrainLeft.set(leftOutput);
	// RobotMap.driveTrainRight.set(rightOutput);
	// }
	//
	// /**
	// * Sets target velocity of each tread in inches / sec
	// *
	// * @param leftOutput
	// * left target velocity
	// * @param rightOutput
	// * right target velocity
	// */
	// public void setLeftRightVelocity(double leftOutput, double rightOutput) {
	//
	// ratePIDLeft.setPID(ConstantsIO.kP_DriveVelocity,
	// ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity,
	// ConstantsIO.kF_DriveVelocity);
	// ratePIDRight.setPID(ConstantsIO.kP_DriveVelocity,
	// ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity,
	// ConstantsIO.kF_DriveVelocity);
	//
	// if (leftOutput != 0) {
	// ratePIDLeft.enable();
	// leftVoltageRamp.enable();
	// ratePIDLeft.setSetpoint(leftOutput);
	// } else {
	// ratePIDLeft.disable();
	// leftVoltageRamp.disable();
	// RobotMap.driveTrainLeft.set(0);
	// }
	//
	// if (rightOutput != 0) {
	// ratePIDRight.enable();
	// rightVoltageRamp.enable();
	// ratePIDRight.setSetpoint(rightOutput);
	// } else {
	// ratePIDRight.disable();
	// rightVoltageRamp.disable();
	// RobotMap.driveTrainRight.set(0);
	// }
	//
	//
	//
	// }
	public void setCurrentModeLeft(boolean isCurrent) {
		isCurrentLeft = isCurrent;
		if (isCurrent) {
			RobotMap.driveLeft1.changeControlMode(TalonControlMode.Current);
			RobotMap.driveLeft2.changeControlMode(TalonControlMode.Current);
			RobotMap.driveLeft3.changeControlMode(TalonControlMode.Current);
		} else {
			RobotMap.driveLeft1.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveLeft2.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveLeft3.changeControlMode(TalonControlMode.PercentVbus);
		}
	}

	public void setCurrentModeRight(boolean isCurrent) {
		isCurrentRight = isCurrent;
		if (isCurrent) {
			RobotMap.driveRight1.changeControlMode(TalonControlMode.Current);
			RobotMap.driveRight2.changeControlMode(TalonControlMode.Current);
			RobotMap.driveRight3.changeControlMode(TalonControlMode.Current);
		} else {
			RobotMap.driveRight1.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveRight2.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveRight3.changeControlMode(TalonControlMode.PercentVbus);
		}
	}

	public boolean isCurrentModeLeft() {
		return isCurrentLeft;
	}

	public boolean isCurrentModeRight() {
		return isCurrentRight;
	}

	public double getCurvature() {
		return Math.abs(RobotMap.driveEncLeft.getRate() + RobotMap.driveEncRight.getRate()) / 2 > MIN_SPEED
				? curvatureSource.pidGet() : 0;
	}

	public double getSteering() {
		return steeringPidController.getSetpoint();
	}

	public double getCurvatureError() {

		return steeringPidController.getAvgError();
	}

	public void updateConstants() {
		steeringPidController.setPID(ConstantsIO.kP_DriveSteering, ConstantsIO.kI_DriveSteering,
				ConstantsIO.kD_DriveSteering, ConstantsIO.kF_DriveSteering);

		throttleRamp.setRampRates(ConstantsIO.kUpRamp_Drive, ConstantsIO.kDownRamp_Drive);
	}

	@Override
	protected void initDefaultCommand() {
		System.out.println("init default");
		setDefaultCommand(new DriveWithControllers(
				DriveWithControllers.useCurrentFlag | DriveWithControllers.hasSteeringCorrectionFlag));
	}

	public void reset() {
		steeringPidController.disable();
	}
}
