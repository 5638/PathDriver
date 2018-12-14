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

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import jaci.pathfinder.modifiers.TankModifier;

public class FollowProfile extends Command {

	ScheduledExecutorService scheduler;
	ScheduledFuture<?> motionFollower;
	TankModifier modifier;
	
    public FollowProfile(TankModifier profile) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.DriveTrain);
    	scheduler = Executors.newScheduledThreadPool(1);
    	modifier = profile;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	Robot.DriveTrain.getLeftFollower().setTrajectory(modifier.getLeftTrajectory());
    	Robot.DriveTrain.getRightFollower().setTrajectory(modifier.getRightTrajectory());
    	
    	Robot.DriveTrain.zeroEncoders();	
    	motionFollower = scheduler.scheduleAtFixedRate(new Runnable(){
    	            	public void run(){
    	            		Robot.DriveTrain.setLeftMotorVelocity(Robot.DriveTrain.getLeftFollower().calculate(Robot.DriveTrain.getLeftEncoderCount()));
    	            		Robot.DriveTrain.setRightMotorVelocity(
    	            				Robot.DriveTrain.getRightFollower().calculate(Robot.DriveTrain.getRightEncoderCount()));
    	            		}
    	            }, 
    	            (int)(10), 
    	            (int)(10), 
    	            TimeUnit.MILLISECONDS);   

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("Left Profile Error", Robot.DriveTrain.getLeftEncoderCount() - Robot.DriveTrain.getLeftFollower().getSegment().position);
    	SmartDashboard.putNumber("Right Profile Error", Robot.DriveTrain.getRightEncoderCount() - Robot.DriveTrain.getRightFollower().getSegment().position);

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.DriveTrain.getLeftFollower().isFinished() && Robot.DriveTrain.getRightFollower().isFinished();
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