package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.PIDOutputWrapper;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.ScalingMax;
import org.usfirst.frc.team2485.util.ThresholdHandler;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @author Nicholas Contreras
 */

public class DriveTrain extends Subsystem {
	public enum DriveSpeed {
		SLOW_SPEED_RATING, NORMAL_SPEED_RATING;

		public double getSpeedFactor() {

			switch (this) {
			case SLOW_SPEED_RATING:
				return 0.5;
			case NORMAL_SPEED_RATING:
				return 1.0;
			default:
				return 1.0;
			}
		}
	}

	private static final double STEERING_DEADBAND = 0.15;
	private static final double THROTTLE_DEADBAND = 0.15;
	private static final boolean USE_GYRO_STEERING_CORRECTION = false;
	private double driveSpeed = DriveSpeed.NORMAL_SPEED_RATING.getSpeedFactor();
	private static double MAX_CURRENT;
	private static double MIN_CURRENT = 0.2;

	private boolean isQuickTurn;
	private boolean isCurrentLeft, isCurrentRight;
	private boolean useCurrent;
	private boolean isAuto;
	private boolean isRotateTo;

	private double oldSteering, negInertiaAccumulator, quickStopAccumulator;

	private WarlordsPIDController velocityPIDRight = new WarlordsPIDController(), 
			velocityPIDLeft = new WarlordsPIDController(), 
			rotateToPID = new WarlordsPIDController(),
			distPID = new WarlordsPIDController(), 
			anglePID = new WarlordsPIDController(),
			steeringPIDController = new WarlordsPIDController();
	private TransferNode rotateToTransferNode = new TransferNode(0), 
			throttleTransferNode = new TransferNode(0), 
			steeringTransferNode = new TransferNode(0),
			overallVelocityTransferNode = new TransferNode(0), 
			angleSteeringTransferNode = new TransferNode(0), 
			autoCurvatureTransferNode = new TransferNode(0);
	private PIDSourceWrapper curvatureSource = new PIDSourceWrapper(), 
			autoSteeringSource = new PIDSourceWrapper(),
			prescaledVelocityLeft = new PIDSourceWrapper(), 
			prescaledVelocityRight = new PIDSourceWrapper(),
			prescaledPowerRight = new PIDSourceWrapper(), 
			prescaledPowerLeft = new PIDSourceWrapper();
	private ScalingMax powerScalingMax = new ScalingMax(), 
			velocityScalingMax = new ScalingMax();
	private RampRate throttleRamp = new RampRate(ConstantsIO.kUpRamp_Drive, ConstantsIO.kDownRamp_Drive), 
			velocityRampLeft = new RampRate(ConstantsIO.kUpRamp_IndividualVelocityRamp, ConstantsIO.kDownRamp_IndividualVelocityRamp), 
			velocityRampRight = new RampRate(ConstantsIO.kUpRamp_IndividualVelocityRamp, ConstantsIO.kDownRamp_IndividualVelocityRamp),
			steeringRamp = new RampRate(ConstantsIO.kUpRamp_DriveSteering, ConstantsIO.kDownRamp_DriveSteering),
			overallVelocityRampRate  = new RampRate(ConstantsIO.kUpRamp_OverallVelocityRamp, ConstantsIO.kDownRamp_OverallVelocityRamp);
	private PIDOutputWrapper motorModeSwitcherLeft = new PIDOutputWrapper(), 
			motorModeSwitcherRight = new PIDOutputWrapper();
	

	private static final double MIN_SPEED = 1;
	private static final double MAX_SPEED = 180;


	private static final double LOW_SPEED_DRIVETO = 1;
	private static final double LOW_SPEED_ROTATETO = .5;
	private static final double DRIVETO_TOLERANCE = 2;
	private static final double ROTATETO_TOLERANCE = .5;

	public DriveTrain() {

		MAX_CURRENT = 1 / ConstantsIO.kF_DriveCurrent;

		motorModeSwitcherLeft.setPidOutput((double out) -> {
			if (Math.abs(out * MAX_CURRENT) > MIN_CURRENT && useCurrent) {
				setCurrentModeLeft(true);
				RobotMap.driveTrainLeft.set(out * MAX_CURRENT);
			} else {
				setCurrentModeLeft(false);
				RobotMap.driveTrainLeft.set(out);
			}
		});
		motorModeSwitcherRight.setPidOutput((double out) -> {
			if (Math.abs(out * MAX_CURRENT) > MIN_CURRENT && useCurrent) {
				setCurrentModeRight(true);
				RobotMap.driveTrainRight.set(out * MAX_CURRENT);
			} else {
				setCurrentModeRight(false);
				RobotMap.driveTrainRight.set(out);
			}
		});

		powerScalingMax.setOutputs(motorModeSwitcherLeft, motorModeSwitcherRight);
		powerScalingMax.setSources(prescaledPowerLeft, prescaledPowerRight);
		powerScalingMax.setSetpoint(1);
		
		prescaledPowerRight.setPidSource(() -> {
			if (isQuickTurn) {
				return -steeringTransferNode.pidGet();
			} else {
				return throttleTransferNode.getOutput() * (1 - steeringTransferNode.getOutput());
			}
		});
		prescaledPowerLeft.setPidSource(()->{
			if (isQuickTurn) {
				return steeringTransferNode.pidGet();
			} else {
				return throttleTransferNode.getOutput() * (1 + steeringTransferNode.getOutput());
			}
		});
		
		steeringPIDController.setSources(curvatureSource);
		steeringPIDController.setOutputs(steeringTransferNode);
		steeringPIDController.setPID(ConstantsIO.kP_DriveSteering, ConstantsIO.kI_DriveSteering,
				ConstantsIO.kD_DriveSteering, ConstantsIO.kF_DriveSteering);
		
		curvatureSource.setPidSource(() -> {
			double leftVelocity = RobotMap.driveEncLeft.getRate();
			double rightVelocity = RobotMap.driveEncRight.getRate();
			if (Math.abs(leftVelocity + rightVelocity) / 2 < MIN_SPEED) {
				steeringPIDController.disable();
				return 0;
			} else if (USE_GYRO_STEERING_CORRECTION) {
				return RobotMap.ahrs.getRate() / ((leftVelocity + rightVelocity) / 2) * RobotMap.ROBOT_WIDTH / 2;
			} else {
				return (leftVelocity - rightVelocity) / (leftVelocity + rightVelocity);
			}

		});
		
		throttleRamp.setOutputs(throttleTransferNode);
		steeringRamp.setOutputs(steeringTransferNode);
		
		//AUTO
		velocityPIDLeft.setSources(RobotMap.driveEncRateLeft);
		velocityPIDLeft.setOutputs(motorModeSwitcherLeft);
		velocityPIDLeft.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, 
				ConstantsIO.kD_DriveVelocity, ConstantsIO.kF_DriveVelocity);
		velocityPIDRight.setSources(RobotMap.driveEncRateRight);
		velocityPIDRight.setOutputs(motorModeSwitcherRight);
		velocityPIDRight.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity,
				ConstantsIO.kD_DriveVelocity, ConstantsIO.kF_DriveVelocity);
		
		velocityRampLeft.setOutputs(velocityPIDLeft);
		velocityRampRight.setOutputs(velocityPIDRight);
		
		velocityScalingMax.setOutputs(velocityRampLeft, velocityRampRight);
		velocityScalingMax.setSources(prescaledVelocityLeft, prescaledVelocityRight);
		velocityScalingMax.setSetpoint(MAX_SPEED);
		
		prescaledVelocityLeft.setPidSource(() -> {
			if (isRotateTo) {
				return rotateToTransferNode.getOutput();
			} else {
				return overallVelocityTransferNode.getOutput() * (1 + autoSteeringSource.pidGet());
			}
		});

		prescaledVelocityRight.setPidSource(() -> {
			if (isRotateTo) {
				return -rotateToTransferNode.getOutput();
			} else {
				return overallVelocityTransferNode.getOutput() * (1 - autoSteeringSource.pidGet());
			}
		});
		
		autoSteeringSource.setPidSource(() -> {
			return angleSteeringTransferNode.getOutput() + autoCurvatureTransferNode.getOutput();
		});
		
		anglePID.setSources(RobotMap.ahrs);
		anglePID.setOutputs(angleSteeringTransferNode);
		anglePID.setPID(ConstantsIO.kP_DriveAngle, ConstantsIO.kI_DriveAngle, ConstantsIO.kD_DriveAngle);
		
		overallVelocityRampRate.setOutputs(overallVelocityTransferNode);
		
		distPID.setSources(RobotMap.averageEncoderDistance);
		distPID.setOutputs(overallVelocityRampRate);
		distPID.setPID(ConstantsIO.kP_Distance, ConstantsIO.kI_Distance, ConstantsIO.kD_Distance,
				ConstantsIO.kF_Distance);
		distPID.setAbsoluteTolerance(DRIVETO_TOLERANCE);
		
		rotateToPID.setSources(RobotMap.ahrs);
		rotateToPID.setOutputs(rotateToTransferNode);
		rotateToPID.setPID(ConstantsIO.kP_RotateTo, ConstantsIO.kI_RotateTo, 
				ConstantsIO.kD_RotateTo, ConstantsIO.kF_RotateTo);
		rotateToPID.setAbsoluteTolerance(ROTATETO_TOLERANCE);
		rotateToPID.setInputRange(0, 360);
		rotateToPID.setOutputRange(-10, 10);
		rotateToPID.setContinuous(true);
				
	}

	public void setAuto(boolean isAuto) {
		this.isAuto = isAuto;
		if (isAuto) {
			velocityPIDLeft.setSetpointSource(null);
			velocityPIDRight.setSetpointSource(null);
		} else {
			velocityPIDLeft.setSetpointSource(new PIDSourceWrapper(() -> {
				return prescaledPowerLeft.pidGet() * MAX_SPEED;
			}));
			velocityPIDRight.setSetpointSource(new PIDSourceWrapper(() -> {
				return prescaledPowerRight.pidGet() * MAX_SPEED;
			}));
		}
	}

	public void setDriveSpeed(DriveSpeed speed) {
		driveSpeed = speed.getSpeedFactor();
	}

	public double getDriveSpeed() {
		return driveSpeed;
	}

	public void setQuickTurn(boolean quickTurn) {
		this.isQuickTurn = quickTurn;
	}

	public void cheesyDrive(double controllerY, double controllerX) {
		double steeringNonLinearity;

		double steering = ThresholdHandler.deadbandAndScale(controllerX, STEERING_DEADBAND, 0.01, 1);
		double throttle = ThresholdHandler.deadbandAndScale(controllerY, THROTTLE_DEADBAND, 0.01, 1);

		double negInertia = steering - oldSteering;
		oldSteering = steering;

		steeringNonLinearity = 0.5;
		// Apply a sin function that's scaled to make it feel better.
		steering = Math.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
				/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);
		steering = Math.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
				/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);
		steering = Math.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
				/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);

		double leftPwm, rightPwm, overPower;
		double sensitivity;

		double angularPower;
		double linearPower;

		// Negative inertia!
		double negInertiaScalar;

		if (steering * negInertia > 0) {
			negInertiaScalar = 2.5;
		} else {
			if (Math.abs(steering) > 0.65) {
				negInertiaScalar = 5.0;
			} else {
				negInertiaScalar = 3.0;
			}
		}
		sensitivity = .75;

		double negInertiaPower = negInertia * negInertiaScalar;
		negInertiaAccumulator += negInertiaPower;

		steering = steering + negInertiaAccumulator;
		if (negInertiaAccumulator > 1) {
			negInertiaAccumulator -= 1;
		} else if (negInertiaAccumulator < -1) {
			negInertiaAccumulator += 1;
		} else {
			negInertiaAccumulator = 0;
		}
		linearPower = throttle * driveSpeed;

		// Quickturn!
		if (isQuickTurn) {
			if (Math.abs(linearPower) < 0.2) {
				double alpha = 0.1;
				quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * steering * 5;
			}
			overPower = 1.0;

			sensitivity = 1.0;

			angularPower = steering;
		} else {
			overPower = 0.0;
			angularPower = Math.abs(throttle) * steering * sensitivity - quickStopAccumulator;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0.0;
			}
		}

		rightPwm = leftPwm = linearPower;
		leftPwm += angularPower;
		rightPwm -= angularPower;

		if (leftPwm > 1.0) {
			rightPwm -= overPower * (leftPwm - 1.0);
			leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
			leftPwm -= overPower * (rightPwm - 1.0);
			rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
			rightPwm += overPower * (-1.0 - leftPwm);
			leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
			leftPwm += overPower * (-1.0 - rightPwm);
			rightPwm = -1.0;
		}

		setCurrentModeLeft(false);
		setCurrentModeRight(false);

		RobotMap.driveTrainLeft.set(leftPwm);
		RobotMap.driveTrainRight.set(rightPwm);

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

		double steering = ThresholdHandler.deadbandAndScale(controllerX, STEERING_DEADBAND, 0.0, 1);
		double throttle = ThresholdHandler.deadbandAndScale(controllerY, THROTTLE_DEADBAND, 0.02, 1);

		throttle *= driveSpeed;
		System.out.println("throttle; " + throttle);
		if (useVelocity && throttle > 0) {
			this.useCurrent = true;
			hasSteeringCorrection = false;
			powerScalingMax.disable();
			velocityPIDLeft.enable();
			velocityPIDRight.enable();
			setAuto(false);
		} else {
			powerScalingMax.enable();
			velocityPIDLeft.disable();
			velocityPIDRight.disable();
			this.useCurrent = useCurrent;
		}

		throttleRamp.enable();
		throttleRamp.setSetpoint(throttle);

		steeringRamp.setSetpoint(steering);
		steeringRamp.enable();

		double averageSpeed = (RobotMap.driveEncLeft.getRate() + RobotMap.driveEncRight.getRate()) / 2;
		if (Math.abs(averageSpeed) > MIN_SPEED && !isQuickTurn && hasSteeringCorrection) {
			steeringPIDController.enable();
			steeringRamp.setOutputs(steeringPIDController);
		} else {
			steeringPIDController.disable();
			steeringRamp.setOutputs(steeringTransferNode);
		}

	}

	public void zeroEncoders() {
		RobotMap.driveEncLeft.reset();
		RobotMap.driveEncRight.reset();

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
		return steeringPIDController.getSetpoint();
	}

	public double getCurvatureError() {

		return steeringPIDController.getAvgError();
	}

	public double getLeftVelocityPIDError() {
		return velocityPIDLeft.getAvgError();
	}

	public double getAnglePIDError() {
		return anglePID.getAvgError();
	}

	public void updateConstants() {
		MAX_CURRENT = 1 / ConstantsIO.kF_DriveCurrent;
		steeringPIDController.setPID(ConstantsIO.kP_DriveSteering, ConstantsIO.kI_DriveSteering,
				ConstantsIO.kD_DriveSteering, ConstantsIO.kF_DriveSteering);
		velocityPIDRight.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity,
				ConstantsIO.kD_DriveVelocity, ConstantsIO.kF_DriveVelocity);
		velocityPIDLeft.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity,
				ConstantsIO.kF_DriveVelocity);
		throttleRamp.setRampRates(ConstantsIO.kUpRamp_Drive, ConstantsIO.kDownRamp_Drive);
		velocityRampLeft.setRampRates(ConstantsIO.kUpRamp_IndividualVelocityRamp,
				ConstantsIO.kDownRamp_IndividualVelocityRamp);
		velocityRampRight.setRampRates(ConstantsIO.kUpRamp_IndividualVelocityRamp,
				ConstantsIO.kDownRamp_IndividualVelocityRamp);
		rotateToPID.setPID(ConstantsIO.kP_RotateTo, ConstantsIO.kI_RotateTo, ConstantsIO.kD_RotateTo,
				ConstantsIO.kF_RotateTo);
		RobotMap.driveLeft1.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		RobotMap.driveLeft2.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		RobotMap.driveLeft3.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		RobotMap.driveRight1.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		RobotMap.driveRight2.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		RobotMap.driveRight3.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		overallVelocityRampRate.setRampRates(ConstantsIO.kUpRamp_OverallVelocityRamp,
				ConstantsIO.kDownRamp_OverallVelocityRamp);
		steeringRamp.setRampRates(ConstantsIO.kUpRamp_DriveSteering, ConstantsIO.kDownRamp_DriveSteering);
		distPID.setPID(ConstantsIO.kP_Distance, ConstantsIO.kI_Distance, ConstantsIO.kD_Distance,
				ConstantsIO.kF_Distance);
		anglePID.setPID(ConstantsIO.kP_DriveAngle, ConstantsIO.kI_DriveAngle, ConstantsIO.kD_DriveAngle);
	}

	@Override
	protected void initDefaultCommand() {
		System.out.println("init default");
		setDefaultCommand(new DriveWithControllers(DriveWithControllers.useVelocityFlag));
		// setDefaultCommand(new
		// DriveWithControllers(DriveWithControllers.useCurrentFlag |
		// DriveWithControllers.hasSteeringCorrectionFlag));
	}

	public void reset() {
		steeringPIDController.disable();
		velocityPIDLeft.disable();
		velocityPIDRight.disable();
		throttleRamp.disable();
		velocityRampLeft.disable();
		velocityRampRight.disable();
		velocityScalingMax.disable();
		overallVelocityRampRate.disable();
		anglePID.disable();
		distPID.disable();
		motorModeSwitcherLeft.pidWrite(0);
		motorModeSwitcherRight.pidWrite(0);
	}

	public void setLeftRightVelocity(double r, double l) {
		useCurrent = false;
		powerScalingMax.disable();
		velocityRampLeft.enable();
		velocityRampRight.enable();
		velocityPIDLeft.enable();
		velocityPIDRight.enable();
		velocityRampLeft.setSetpoint(l);
		velocityRampRight.setSetpoint(r);
	}

	public boolean driveTo(double distance, double maxSpeed, double angle, double curvature) {
		useCurrent = true;
		powerScalingMax.disable();
		velocityRampLeft.enable();
		velocityRampRight.enable();
		velocityPIDLeft.enable();
		velocityPIDRight.enable();
		distPID.enable();
		isRotateTo = false;
		velocityScalingMax.enable();
		overallVelocityRampRate.enable();
		distPID.setSetpoint(distance);
		distPID.setOutputRange(-maxSpeed, maxSpeed);
		anglePID.enable();
		anglePID.setSetpoint(angle);
		return (distPID.isOnTarget() && Math
				.abs((RobotMap.driveEncRight.getRate() + RobotMap.driveEncLeft.getRate()) / 2) < LOW_SPEED_DRIVETO);

	}

	public boolean rotateTo(double angle) {
		rotateToPID.enable();
		rotateToPID.setSetpoint(angle);
		isRotateTo = true;
		setAuto(true);
		useCurrent = true;
		powerScalingMax.disable();
		velocityRampLeft.enable();
		velocityRampRight.enable();
		velocityPIDLeft.enable();
		velocityPIDRight.enable();
		velocityScalingMax.enable();
		anglePID.enable();
		anglePID.setSetpoint(angle);
		return (rotateToPID.isOnTarget() && Math.abs(RobotMap.ahrs.getRate()) < LOW_SPEED_ROTATETO);
	}
}
