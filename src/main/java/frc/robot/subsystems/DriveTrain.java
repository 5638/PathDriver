package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;
import frc.robot.commands.DriveCom;
import jaci.pathfinder.followers.DistanceFollower;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveTrain extends Subsystem {

  private final TalonSRX r1 = RobotMap.r1;
  private final TalonSRX l1 = RobotMap.l1;
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






  public DriveTrain() {
    leftDriveFollower = new DistanceFollower();
    rightDriveFollower = new DistanceFollower();
  }

  public DistanceFollower getLeftFollower(){
    return this.leftDriveFollower;
  }
  public DistanceFollower getRightFollower(){
    return this.rightDriveFollower;
  }
  public void zeroEncoders(){
    this.l1.setSelectedSensorPosition(0, 0, 10);
    this.r1.setSelectedSensorPosition(0, 0, 10);
  }
  public double getLeftEncoderCount(){
    return this.l1.getSelectedSensorPosition(0);
  }
  public double getLeftEncoderVelocity(){
    return this.l1.getSelectedSensorVelocity(0);
  }
  public double getRightEncoderCount(){
    return this.r1.getSelectedSensorPosition(0);
  }
  public double getRightEncoderVelocity(){
    return this.r1.getSelectedSensorVelocity(0);
  }
  public void setLeftMotorVelocity(double speed){
    this.l1.set(ControlMode.PercentOutput, speed);
  }
  public void setRightMotorVelocity(double speed){
    this.r1.set(ControlMode.PercentOutput, speed);
  }







  public void drive(XboxController x0) {
    double steer = x0.getRawAxis(0);
    double a = x0.getRawAxis(3);
    double b = x0.getRawAxis(2);

    double throttle = a - b;

    driveTrain.arcadeDrive(throttle, steer);
    
    //gets left and right side velocity
   // SmartDashboard.putNumber("Left Motor", RobotMap.l1.getSelectedSensorVelocity(0));  
   // SmartDashboard.putNumber("Right Motor", RobotMap.r1.getSelectedSensorVelocity(0));
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