package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.constants;

import frc.robot.Robot;

public class RobotMap {

	//drive
	public static WPI_TalonSRX l1;
	public static WPI_VictorSPX l2;
	public static SpeedControllerGroup lg;

	public static WPI_TalonSRX r1;
	public static WPI_VictorSPX r2;
	public static SpeedControllerGroup rg;

	public static DifferentialDrive dt;

	//gyro
	public static double a;
	public static AHRS gyro;

	//shift
	public static DoubleSolenoid shift;

	//elevator
	public static WPI_TalonSRX el;

	//intake
	public static WPI_VictorSPX in1;
	public static WPI_VictorSPX in2;
	//clamp
	public static DoubleSolenoid c;

	//dump
	public static WPI_TalonSRX d;

	public int rpos;
	public int lpos;
	

	public static void init() {
		//left side
		WPI_TalonSRX l1 = new WPI_TalonSRX(1);		//left master
		l1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		l1.setSensorPhase(true);
		l1.setSafetyEnabled(false);
		l1.configNominalOutputForward(0, 10);
	 	l1.configNominalOutputReverse(0, 10);
	 	l1.configPeakOutputForward(1, 10);
		l1.configPeakOutputReverse(-1, 10);
		l1.configOpenloopRamp(.5, 10); 

		l1.config_kP(0, constants.dtkP, constants.t);
		l1.config_kI(0, constants.dtkI, constants.t);
		l1.config_kD(0, constants.dtkD, constants.t);
		l1.config_kF(0, constants.dtkF, constants.t);

		WPI_VictorSPX l2 = new WPI_VictorSPX(2); 	//left slave
		l2.follow(l1);

		//group left
		lg = new SpeedControllerGroup(l1, l2);

		//right side
		WPI_TalonSRX r1 = new WPI_TalonSRX(3); 		//right master
		r1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		r1.setSensorPhase(true);
		r1.setSafetyEnabled(false);
		r1.configNominalOutputForward(0, 10);
		r1.configNominalOutputReverse(0, 10);
		r1.configPeakOutputForward(1, 10);
		r1.configPeakOutputReverse(-1, 10);
		r1.configOpenloopRamp(.5, 10);

		r1.config_kP(0, constants.dtkP, constants.t);
		r1.config_kI(0, constants.dtkI, constants.t);
		r1.config_kD(0, constants.dtkD, constants.t);
		r1.config_kF(0, constants.dtkF, constants.t);

		WPI_VictorSPX r2 = new WPI_VictorSPX(4); 	//right slave
		r2.follow(r1);

		//group right
		rg = new SpeedControllerGroup(r1, r2);

		//finished drive
		dt = new DifferentialDrive(lg, rg);
		shift = new DoubleSolenoid(0, 0, 1);

		//gyro
		gyro = Robot.gyro;


		//elevator
		/*
		WPI_TalonSRX el = new WPI_TalonSRX(5); 		//elevator talon
		el.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		el.setSensorPhase(true);
		el.setSafetyEnabled(false);
		el.configNominalOutputForward(0, 10);
		el.configNominalOutputReverse(0, 10);
		el.configPeakOutputForward(1, 10);
		el.configPeakOutputReverse(-1, 10);
		el.configOpenloopRamp(.5, 10);
		el.config_kP(0, constants.ekP, constants.t);
		el.config_kI(0, constants.ekP, constants.t);
		el.config_kD(0, constants.ekP, constants.t);
		el.config_kF(0, constants.ekP, constants.t);

*/


		//intake

		/*
		WPI_VictorSPX in1 = new WPI_VictorSPX(6); 	//intake victor 1
		WPI_VictorSPX in2 = new WPI_VictorSPX(7);	//intake victor 2

		c = new DoubleSolenoid(0, 2, 3);

*/



/*
		//dump
		WPI_TalonSRX d = new WPI_TalonSRX(8);		//dump talon
		d.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		el.configNominalOutputForward(0, 10);
		el.configNominalOutputReverse(0, 10);
		el.configPeakOutputForward(1, 10);
		el.configPeakOutputReverse(-1, 10);
		el.configOpenloopRamp(.5, 10);
		el.config_kP(0, constants.dkP, constants.t);
		el.config_kI(0, constants.dkI, constants.t);
		el.config_kD(0, constants.dkD, constants.t);
		el.config_kF(0, constants.dkF, constants.t);
	}
	*/
}
}