// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterWPIPID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /** Creates a new Autonomous. */
  //Timer m_timer = new Timer();
  public Autonomous(Drivetrain drive, ShooterWPIPID shoot, Hopper hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
    new autoDrive(drive, 1, .6),
    new autoShoot(shoot, hopper, 2, .4),
    new autoDrive(drive, 5, -.6))
  
    //new InstantCommand(() -> shoot.setSpeed(ShooterConstants.shootSpeed), shoot).withTimeout(5)
      /*
      new TurnDegreesPID(120, drive),
      //new RunCommand(() -> m_timer.delay(1)),
      new DriveSetDistance(5, drive),
      //new RunCommand(() -> m_timer.delay(1)),
      new TurnDegreesPID(20, drive),
      //new RunCommand(() -> m_timer.delay(1)),
      new RunCommand(() -> shoot.setSpeed(.5), shoot)
      */
    );
  }
}
