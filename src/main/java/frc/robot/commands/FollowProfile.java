/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import jaci.pathfinder.modifiers.TankModifier;

public class FollowProfile extends Command {

	private WPI_TalonSRX r1 = RobotMap.r1;
  	private WPI_TalonSRX l1 = RobotMap.l1;

	ScheduledExecutorService scheduler;
	ScheduledFuture<?> motionFollower;
	TankModifier modifier;


	public FollowProfile(TankModifier profile){
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		requires(Robot.Sensors);
		//setDefaultCommand(new Path1());
    	scheduler = Executors.newScheduledThreadPool(1);
    	modifier = profile;
	}

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	Robot.Sensors.getLeftFollower().setTrajectory(modifier.getLeftTrajectory());
		Robot.Sensors.getRightFollower().setTrajectory(modifier.getRightTrajectory());
		
    	Robot.Sensors.zeroEncoders();	
    	motionFollower = scheduler.scheduleAtFixedRate(new Runnable(){
    	            	public void run(){
    	            		Robot.Sensors.setLeftMotorVelocity(
						Robot.Sensors.getLeftFollower().calculate(Robot.Sensors.getLeftEncoderCount()));
    	            		Robot.Sensors.setRightMotorVelocity(
    	            				Robot.Sensors.getRightFollower().calculate(Robot.Sensors.getRightEncoderCount()));
    	            		}
    	            }, 
    	            (int)(10), 
    	            (int)(10), 
    	            TimeUnit.MILLISECONDS);   

    }

    // Called repeatedly when this Command is scheduled to run
	protected void execute() {
    	SmartDashboard.putNumber("Left Profile Error", Robot.Sensors.getLeftEncoderCount() - Robot.Sensors.getLeftFollower().getSegment().position);
    	SmartDashboard.putNumber("Right Profile Error", Robot.Sensors.getRightEncoderCount() - Robot.Sensors.getRightFollower().getSegment().position);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.Sensors.getLeftFollower().isFinished() && Robot.Sensors.getRightFollower().isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
		motionFollower.cancel(true);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		motionFollower.cancel(true);

    }

	
}