package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.subsystems.LeftSwitch;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.AutoCom;


public class Robot extends TimedRobot {
	public static AHRS gyro;
	
	
	public static DriveTrain DriveTrain = new DriveTrain();
	public static LeftSwitch leftSwitch = new LeftSwitch();
	public static OI OI;

	Command DriveCom;
	Command AutoCom;
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<Command>();


	@Override
	public void robotInit() {
		OI = new OI();
		
		try {
	          gyro = new AHRS(SPI.Port.kMXP); //Initialize NavX Gyro
	      } catch (RuntimeException ex) {
	          DriverStation.reportError("Error instantiating the gyro:  " + ex.getMessage(), true);
	    }
		
		m_chooser.addDefault("Default Auto", new AutoCom());
		SmartDashboard.putData("Auto mode", m_chooser);
	}

	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
