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

	public static void init() {
		//left side
		WPI_TalonSRX l1 = new WPI_TalonSRX(1);		//left master
		l1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		l1.setSelectedSensorPosition(0, 0, 10);
		System.out.println("LEFT SET TO 0");
		System.out.println(l1.getSelectedSensorPosition(0));

		WPI_VictorSPX l2 = new WPI_VictorSPX(2); 	//left slave
		l2.follow(l1);

		//group left
		lg = new SpeedControllerGroup(l1, l2);

		//right side
		WPI_TalonSRX r1 = new WPI_TalonSRX(3); 		//right master
		r1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		r1.setSelectedSensorPosition(0, 0, 10);
		System.out.println("RIGHT SET TO 0");
		System.out.println(r1.getSelectedSensorPosition(0));

		WPI_VictorSPX r2 = new WPI_VictorSPX(4); 	//right slave
		r2.follow(r1);

		//group right
		rg = new SpeedControllerGroup(r1, r2);

		//finished drive
		dt = new DifferentialDrive(lg, rg);
		shift = new DoubleSolenoid(0, 0, 1);

		//gyro
		gyro = Robot.gyro;

		System.out.println("RobotMap Successfully Initiated.");
	}
}