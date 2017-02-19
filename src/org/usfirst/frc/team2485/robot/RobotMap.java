package org.usfirst.frc.team2485.robot;
import org.usfirst.frc.team2485.subsystems.Climber;
import org.usfirst.frc.team2485.subsystems.DriveTrain;
import org.usfirst.frc.team2485.subsystems.Feeder;
import org.usfirst.frc.team2485.subsystems.GearHolder;
import org.usfirst.frc.team2485.subsystems.IntakeArm;
import org.usfirst.frc.team2485.subsystems.IntakeRollers;
import org.usfirst.frc.team2485.subsystems.Shooter;
import org.usfirst.frc.team2485.subsystems.WheelOfDeath;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.MultipleEncoderWrapper;
import org.usfirst.frc.team2485.util.MultipleEncoderWrapper.MultipleEncoderWrapperMode;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
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
	
//	public static Relay compressorSpike;
//	public static DigitalInput pressureSwitch;
	 
	//constants
	public static final double ROBOT_WIDTH = 27;
	public static final double WHEEL_RADIUS = 2;
	public static int driveRightPortCIM1 = 1;
	public static int driveRightPortCIM2 = 3;
	public static int driveRightPortMiniCIM = 2;
	public static int driveLeftPortCIM1 = 4;
	public static int driveLeftPortCIM2 = 6;
	public static int driveLeftPortMiniCIM = 5;
	public static int wheelOfDeathMotorPort=7;
	public static int kShooterEncoderPortA = 4, kShooterEncoderPortB = 5;
	public static int kLeftDriveEnc1 = 3, kLeftDriveEnc2 = 2;
	public static int kRightDriveEnc1 = 0, kRightDriveEnc2 = 1;
	public static int kShooterMotorPort = 0;
	public static int kIntakeMotorPort = 1;
	public static int kFeederEncoderPortA = 6, kFeederEncoderPortB = 7;
	public static int kFeederMotorPort = 2;
	public static int kClimberMotorPort = 7;
	

	

	// speed controllers
	public static SpeedControllerWrapper driveTrainRight, driveTrainLeft;
	public static CANTalon driveLeft1, driveLeft2, driveLeftMini, driveRight1, driveRight2, driveRightMini;
	public static CANTalon deathMotor;
	public static SpeedControllerWrapper shooterMotor;		
	public static SpeedControllerWrapper intakeMotor;
	public static SpeedControllerWrapper feederMotor;
	public static SpeedControllerWrapper climberMotor;
	
	//solenoids
	public static Solenoid gearSolenoidBottom1, gearSolenoidBottom2;
	public static Solenoid gearSolenoidTop1, gearSolenoidTop2;
	public static Solenoid intakeArmSolenoid;

	//sensors
	public static Encoder driveEncLeft, driveEncRight;
	public static EncoderWrapperRateAndDistance driveEncRateLeft, driveEncRateRight;
	public static Encoder shooterEncoder;	
	public static Encoder feederEncoder;
	public static Ultrasonic gearDetector;
	public static MultipleEncoderWrapper averageEncoderDistance;
	public static AHRS ahrs;
	
	// subsystems
	public static GearHolder gearHolder;
	public static DriveTrain driveTrain;
	public static Shooter shooter;
	public static IntakeRollers intakeRollers;
	public static IntakeArm intakeArm;
	public static Climber climber;
	public static WheelOfDeath wheelOfDeath;
	public static Feeder feeder;
	public static Relay lightSpike;
	
	public static void init() {
		
		
		// construct hardware
//		compressorSpike = new Relay(0);
//		pressureSwitch = new DigitalInput(10);
		
		driveLeft1 = new CANTalon(driveLeftPortCIM1);
		driveLeft2 = new CANTalon(driveLeftPortCIM2);
		driveLeftMini = new CANTalon(driveLeftPortMiniCIM);
		
		driveRight1 = new CANTalon(driveRightPortCIM1);
		driveRight2 = new CANTalon(driveRightPortCIM2);
		driveRightMini = new CANTalon(driveRightPortMiniCIM);
		
		driveTrainLeft = new SpeedControllerWrapper(driveLeft1, driveLeft2, driveLeftMini);
		driveTrainRight = new SpeedControllerWrapper(driveRight1, driveRight2, driveRightMini);
		
		deathMotor = new CANTalon(wheelOfDeathMotorPort);
		deathMotor.changeControlMode(TalonControlMode.Speed);
		
//		driveEncLeft = new Encoder(2, 3);
//		driveEncRateLeft = new EncoderWrapperRateAndDistance(driveEncLeft, PIDSourceType.kRate);
//		driveEncRight = new Encoder(0, 1);
//		driveEncRateRight = new EncoderWrapperRateAndDistance(driveEncRight, PIDSourceType.kRate);
		
		shooterMotor = new SpeedControllerWrapper(new VictorSP(kShooterMotorPort));
		intakeMotor = new SpeedControllerWrapper(new VictorSP(kIntakeMotorPort));
		feederMotor = new SpeedControllerWrapper(new VictorSP(kFeederMotorPort));
		climberMotor = new SpeedControllerWrapper(new VictorSP(kClimberMotorPort));
		
		shooterEncoder = new Encoder(kShooterEncoderPortA, kShooterEncoderPortB);
		feederEncoder = new Encoder(kFeederEncoderPortA, kFeederEncoderPortB);
		
		lightSpike = new Relay(0);
		
//		usbCam = new UsbCamera("cam0", 0);
				
		
			

				
//		driveTrainLeft = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(4), new VictorSP(5), new VictorSP(6)});
//		driveTrainRight = new SpeedControllerWrapper(new SpeedController[] {new VictorSP(0), new VictorSP(2), new VictorSP(1)});

		gearSolenoidBottom1 = new Solenoid(0);
		gearSolenoidBottom2 = new Solenoid(1);
		gearSolenoidTop1 = new Solenoid(3);
		gearSolenoidTop2 = new Solenoid(4); 
		intakeArmSolenoid = new Solenoid(2);
						
		
		//configure hardware
//		driveLeft1.changeControlMode(TalonControlMode.Current);
//		driveLeft2.changeControlMode(TalonControlMode.Current);
//		driveLeft3.changeControlMode(TalonControlMode.Current);
//		
//		driveRight1.changeControlMode(TalonControlMode.Current);
//		driveRight2.changeControlMode(TalonControlMode.Current);
//		driveRight3.changeControlMode(TalonControlMode.Current);
		
		driveTrainLeft.setInverted(true);
		
		driveEncLeft = new Encoder(kLeftDriveEnc1, kLeftDriveEnc2);
		driveEncRight = new Encoder(kRightDriveEnc1, kRightDriveEnc2);
		
		
		driveEncLeft.setDistancePerPulse((double)1/250 * (Math.PI * WHEEL_RADIUS * 2));
		driveEncRight.setDistancePerPulse((double)1/250 * (Math.PI * WHEEL_RADIUS * 2));
		
		driveEncRateLeft = new EncoderWrapperRateAndDistance(driveEncLeft, PIDSourceType.kRate);
		driveEncRateRight = new EncoderWrapperRateAndDistance(driveEncRight, PIDSourceType.kRate);
		
		averageEncoderDistance = new MultipleEncoderWrapper(PIDSourceType.kDisplacement, 
				MultipleEncoderWrapperMode.AVERAGE, driveEncLeft, driveEncRight);
		ahrs = new AHRS(Port.kMXP);
		//construct subsystems
		driveTrain = new DriveTrain();
		gearHolder = new GearHolder();
		
		intakeRollers = new IntakeRollers();
		intakeArm = new IntakeArm();
		feeder = new Feeder();
		shooter = new Shooter(); 
		climber = new Climber();
		wheelOfDeath = new WheelOfDeath();
		
		

	}

	public static void updateConstants() {
		
		driveTrain.updateConstants();
		
	}
}
