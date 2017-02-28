package org.usfirst.frc.team2485.robot;

import org.usfirst.frc.team2485.subsystems.Climber;
import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.subsystems.Feeder;
import org.usfirst.frc.team2485.subsystems.GearHolder;
import org.usfirst.frc.team2485.subsystems.IntakeArm;
import org.usfirst.frc.team2485.subsystems.IntakeRollers;
import org.usfirst.frc.team2485.subsystems.Shooter;
import org.usfirst.frc.team2485.subsystems.WheelOfDeath;
import org.usfirst.frc.team2485.util.AHRSWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.MultipleEncoderWrapper;
import org.usfirst.frc.team2485.util.MultipleEncoderWrapper.MultipleEncoderWrapperMode;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;
import org.usfirst.frc.team2485.util.AHRSWrapperRateAndAngle.Units;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// public static Relay compressorSpike;
	// public static DigitalInput pressureSwitch;

	// constants
	public static final double ROBOT_WIDTH = 27;
	public static final double WHEEL_RADIUS = 2;
	public static final double FEEDER_RADIUS = 0.5;

	public static int driveRightPortCIM1 = 1;
	public static int driveRightPortCIM2 = 3;
	public static int driveRightPortMiniCIM = 2;
	public static int driveLeftPortCIM1 = 4;
	public static int driveLeftPortCIM2 = 6;
	public static int driveLeftPortMiniCIM = 5;
	public static int wheelOfDeathMotorPort = 7;
	public static int kShooterEncoderPortA = 8, kShooterEncoderPortB = 9;
	public static int kLeftDriveEnc1 = 3, kLeftDriveEnc2 = 2;
	public static int kRightDriveEnc1 = 0, kRightDriveEnc2 = 1;
	public static int kShooterMotorPort1 = 8, kShooterMotorPort2 = 9;
	public static int kIntakeMotorPort = 3;
	public static int kFeederEncoderPortA = 6, kFeederEncoderPortB = 7;
	public static int kFeederMotorPort = 6;
	public static int kClimberMotorPort = 7;
	public static int kSWODEncoderPort1 = 4;
	public static int kSWODEncoderPort2 = 5;

	// speed controllers
	public static SpeedControllerWrapper driveTrainRight, driveTrainLeft;
	public static CANTalon driveLeft1, driveLeft2, driveRight1, driveRight2;
	public static CANTalon deathMotor;
	public static SpeedControllerWrapper shooterMotors;
	public static SpeedControllerWrapper intakeMotor;
	public static SpeedControllerWrapper feederMotor;
	public static SpeedControllerWrapper climberMotor;
	
	// relays
	public static Relay lightSpike;

	// solenoids
	public static Solenoid gearSolenoidBottom;
	public static Solenoid gearSolenoidTop;
	public static Solenoid intakeArmSolenoidHorizontal;
	public static Solenoid intakeArmSolenoidVertical1;
	public static Solenoid intakeArmSolenoidVertical2;

	// sensors
	public static Encoder driveEncLeft, driveEncRight;
	public static EncoderWrapperRateAndDistance driveEncRateLeft, driveEncRateRight;
	public static Encoder shooterEncoder;
	public static Encoder feederEncoder;
	public static Ultrasonic gearDetector;
	public static MultipleEncoderWrapper averageEncoderDistance, averageEncoderRate;

	public static AHRS ahrs;
	public static AHRSWrapperRateAndAngle ahrsRateRads;
	public static UsbCamera usbCam;

	// subsystems
	public static GearHolder gearHolder;
	public static DriveTrain driveTrain;
	public static Shooter shooter;
	public static IntakeRollers intakeRollers;
	public static IntakeArm intakeArm;
	public static Climber climber;
	public static WheelOfDeath wheelOfDeath;
	public static Feeder feeder;

	public static Compressor compressor;
	public static Encoder swodEncoder;

	public static void init() {

		// CONSTRUCT HARDWARE
		// compressorSpike = new Relay(0);
		// pressureSwitch = new DigitalInput(10);

		// ACTUATORS
		driveLeft1 = new CANTalon(driveLeftPortCIM1);
		driveLeft2 = new CANTalon(driveLeftPortCIM2);

		driveRight1 = new CANTalon(driveRightPortCIM1);
		driveRight2 = new CANTalon(driveRightPortCIM2);

		driveTrainLeft = new SpeedControllerWrapper(driveLeft1, driveLeft2);
		driveTrainRight = new SpeedControllerWrapper(driveRight1, driveRight2);

		deathMotor = new CANTalon(wheelOfDeathMotorPort);
		deathMotor.setInverted(true);
		deathMotor.changeControlMode(TalonControlMode.PercentVbus);

		shooterMotors = new SpeedControllerWrapper(new VictorSP(kShooterMotorPort1), new VictorSP(kShooterMotorPort2));
		intakeMotor = new SpeedControllerWrapper(new VictorSP(kIntakeMotorPort));
		feederMotor = new SpeedControllerWrapper(new VictorSP(kFeederMotorPort));
		climberMotor = new SpeedControllerWrapper(new VictorSP(kClimberMotorPort));


		lightSpike = new Relay(0);

		gearSolenoidBottom = new Solenoid(1);
		gearSolenoidTop = new Solenoid(0);
		intakeArmSolenoidHorizontal = new Solenoid(2);
		intakeArmSolenoidVertical1 = new Solenoid(5);
		intakeArmSolenoidVertical2 = new Solenoid(3);

		// SENSORS
		ahrs = new AHRS(Port.kMXP);

		shooterEncoder = new Encoder(kShooterEncoderPortA, kShooterEncoderPortB);
		swodEncoder = new Encoder(kSWODEncoderPort1, kSWODEncoderPort2);
		feederEncoder = new Encoder(kFeederEncoderPortA, kFeederEncoderPortB);
		
		driveEncLeft = new Encoder(kLeftDriveEnc1, kLeftDriveEnc2);
		driveEncRight = new Encoder(kRightDriveEnc1, kRightDriveEnc2);

		driveEncRateLeft = new EncoderWrapperRateAndDistance(driveEncLeft, PIDSourceType.kRate);
		driveEncRateRight = new EncoderWrapperRateAndDistance(driveEncRight, PIDSourceType.kRate);

		averageEncoderDistance = new MultipleEncoderWrapper(PIDSourceType.kDisplacement,
				MultipleEncoderWrapperMode.AVERAGE, driveEncLeft, driveEncRight);

		averageEncoderRate = new MultipleEncoderWrapper(PIDSourceType.kRate, MultipleEncoderWrapperMode.AVERAGE,
				driveEncLeft, driveEncRight);
		ahrsRateRads = new AHRSWrapperRateAndAngle(PIDSourceType.kRate, Units.RADS);

		gearDetector = new Ultrasonic(14, 15);
		gearDetector.setAutomaticMode(true);
		// usbCam = CameraServer.getInstance().startAutomaticCapture();

		// CONFIGURE HARDWARE

		driveLeft1.setInverted(true);
		driveLeft2.setInverted(true);

		driveEncLeft.setDistancePerPulse((double) 1 / 250 * (Math.PI * WHEEL_RADIUS * 2));
		driveEncRight.setDistancePerPulse((double) 1 / 250 * (Math.PI * WHEEL_RADIUS * 2));
		
		shooterEncoder.setDistancePerPulse(1/250.0);
		shooterEncoder.setPIDSourceType(PIDSourceType.kRate);
		
		swodEncoder.setDistancePerPulse(1.0 / 250);
		swodEncoder.setPIDSourceType(PIDSourceType.kRate);
		
		feederEncoder.setDistancePerPulse((1.0/250)*Math.PI*2*FEEDER_RADIUS);
		feederEncoder.setPIDSourceType(PIDSourceType.kRate);


		// CONSTRUCT SUBSYSTEMS

		driveTrain = new DriveTrain();
		gearHolder = new GearHolder();
		intakeRollers = new IntakeRollers();
		intakeArm = new IntakeArm();
		feeder = new Feeder();
		shooter = new Shooter();
		climber = new Climber();
		wheelOfDeath = new WheelOfDeath();

		compressor = new Compressor();

	}

	public static void updateConstants() {
		wheelOfDeath.updateConstants();
		driveTrain.updateConstants();
		shooter.updateConstants();
		feeder.updateConstants();

	}
}
