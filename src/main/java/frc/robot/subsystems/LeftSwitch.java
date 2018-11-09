/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.constants;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Add your docs here.
 */
public class LeftSwitch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static double l_, r_;

  public static boolean done = false;

  
  public void path() {
    int rightEnc = RobotMap.r1.getSelectedSensorPosition(0);
    int leftEnc = RobotMap.l1.getSelectedSensorPosition(0);
    
    
    Waypoint[] leftSwitchWaypoint = new Waypoint[] { 
      //X,Y,Tangent X,Tangent Y,Fixed Theta,Name
      //6.752066115702479,5.756749311294766,10.0,0.0,true,
      //16.778786099396786,8.02034733714123,0.0,10.0,true,
  
      new Waypoint(6.752066115702479, 5.756749311294766, 0),
      new Waypoint(16.778786099396786, 8.02034733714123, 0.0)
    };


      Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.010, 2.67, 2, 60);

      Trajectory trajectory = Pathfinder.generate(leftSwitchWaypoint, config);

      TankModifier modifier = new TankModifier(trajectory).modify(0.55245);



      EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
      EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());

      left.configureEncoder(leftEnc, 4096, 0.1524);
      left.configurePIDVA(constants.dtkP, 0.0, constants.dtkD, 1 / 2.67, 1);  // The first argument is the proportional gain. Usually this will be quite high
                                                                              // The second argument is the integral gain. This is unused for motion profiling
                                                                              // The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
                                                                              // The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
                                                                              //      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
                                                                              // The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker

      right.configureEncoder(rightEnc, 4096, 0.1524);
      right.configurePIDVA(constants.dtkP, 0.0, constants.dtkD, 1 / 2.67, 1);

      double l = left.calculate(leftEnc);
      double r = right.calculate(rightEnc);

      double gyro_heading = Robot.gyro.getAngle();
      double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

      double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
      double turn = 0.8 * (-1.0/80.0) * angleDifference;

      RobotMap.l1.set(l + turn);
      RobotMap.r1.set(r - turn);
      

      r_ = r - turn;
      l_ = l + turn;

      if(left.isFinished() && right.isFinished()){
        done = true;
      }

      
}

  
  public boolean isPathComplete() {
    if(done == true){
      return true;
    }else{
      return false;
    }
  }

}


