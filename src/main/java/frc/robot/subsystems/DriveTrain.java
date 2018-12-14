package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCom;
import jaci.pathfinder.followers.DistanceFollower;

public class DriveTrain extends Subsystem {

  private final WPI_TalonSRX r1 = RobotMap.r1;
  private final WPI_TalonSRX l1 = RobotMap.l1;
  private final DifferentialDrive driveTrain = RobotMap.dt;
  private final DoubleSolenoid shift = RobotMap.shift;
  PIDController heading;
  double RotateToAngleRate;
  boolean RotateToAngle = false;
  double currentRotationRate;

  DistanceFollower leftDriveFollower;
  DistanceFollower rightDriveFollower;


  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveCom());
  }


  public void drive(XboxController x0) {
    double steer = x0.getRawAxis(0);
    double a = x0.getRawAxis(3);
    double b = x0.getRawAxis(2);

    double throttle = a - b;

    driveTrain.arcadeDrive(throttle, steer);
    
    //gets left and right side velocity
    SmartDashboard.putNumber("Left Motor", l1.getSelectedSensorVelocity(0));  
    SmartDashboard.putNumber("Right Motor", r1.getSelectedSensorVelocity(0));
  }

  public void stop() {
    driveTrain.arcadeDrive(0, 0);
  }

  public void shiftHI() {
    shift.set(Value.kForward);
  }

  public void shiftLO() {
    shift.set(Value.kReverse);
  }
  
}