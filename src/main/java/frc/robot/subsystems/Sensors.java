/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import jaci.pathfinder.followers.DistanceFollower;

/**
 * Add your docs here.
 */
public class Sensors extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX r1 = RobotMap.r1;
  private TalonSRX l1 = RobotMap.l1;
  private DifferentialDrive driveTrain = RobotMap.dt;
  private DoubleSolenoid shift = RobotMap.shift;
  PIDController heading;
  double RotateToAngleRate;
  boolean RotateToAngle = false;
  double currentRotationRate;

  DistanceFollower leftDriveFollower;
  DistanceFollower rightDriveFollower;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public Sensors(){
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
  
}
