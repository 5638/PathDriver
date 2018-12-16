/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.*;
import jaci.pathfinder.modifiers.*;

public class LeftSwitchAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftSwitchAuto() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
    addSequential(new AutoCom());
    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    Waypoint[] points =  new Waypoint[] {
      new Waypoint(0, 0, 0),
      new Waypoint(104.5, 16.35, Pathfinder.d2r(60))
      };
        Config config = new Config(
      Trajectory.FitMethod.HERMITE_CUBIC,
      Trajectory.Config.SAMPLES_HIGH, RobotMap.PROFILE_DT, 30.0, 20.0, 240.0);
        TankModifier profile = new TankModifier(Pathfinder.generate(points, config)).modify(28);
        
        addSequential(new FollowProfile(profile));
        
  }
}
