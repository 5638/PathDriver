
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.AutoCom;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.*;
import jaci.pathfinder.modifiers.TankModifier;


/**
 * Add your docs here.
 */

  // Put methods for controlling this subsystem
  // here. Call these from Commands.



public class MotionProfileExample extends Subsystem {

	@Override
  	public void initDefaultCommand() {
    	// Set the default command for a subsystem here.
    	setDefaultCommand(new AutoCom());
 	}

	public void DoProfile(){

		double wheelbase_width = 0.54;

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 2, 1.0, 60.0);
			Waypoint[] points = new Waypoint[] {
				//new Waypoint(-4, -1, Pathfinder.d2r(-45)),
				//new Waypoint(-2, -2, 0),
				//new Waypoint(0, 0, 0)
				new Waypoint(0, 0, 0),
				new Waypoint(2, 0, 0)
			};

		Trajectory trajectory = Pathfinder.generate(points, config);

		// Wheelbase Width = 0.5m
		TankModifier modifier = new TankModifier(trajectory);
		modifier.modify(wheelbase_width);

		// Do something with the new Trajectories...
		EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
		EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());

		left.configureEncoder(RobotMap.l1.getSelectedSensorPosition(0), 4096, 0.1524);
		right.configureEncoder(RobotMap.r1.getSelectedSensorPosition(0), 4096, 0.1524);

		left.configurePIDVA(1, 0, 0, 1/2, 0);
		right.configurePIDVA(1, 0, 0, 1/2, 0);

		for(int i = 0; i < trajectory.length(); i++){
			Trajectory.Segment seg = trajectory.get(i);

			System.out.printf(	
				"%f,%f,%f,%f,%f,%f,%f,%f\n", 
        		seg.dt, seg.x, seg.y, seg.position, seg.velocity, 
           		seg.acceleration, seg.jerk, seg.heading);
		}

		double outputLeft = left.calculate(RobotMap.l1.getSelectedSensorPosition(0));
		double outputRight = right.calculate(RobotMap.r1.getSelectedSensorPosition(0));

		RobotMap.l1.set(ControlMode.PercentOutput, outputLeft);
		RobotMap.r1.set(ControlMode.PercentOutput, outputRight);
	}


}


