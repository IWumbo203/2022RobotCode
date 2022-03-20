// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterWPIPID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathWeaverAuto extends SequentialCommandGroup {
  /** Creates a new PathWeaverAuto. */
  public PathWeaverAuto(RobotContainer m_robotContainer, Intake intake, Drivetrain drive, Hopper hopper, ShooterWPIPID shoot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup( m_robotContainer.getAutonomousCommand(), 
          new RunCommand(() -> intake.setIntake(IntakeConstants.intakeSpeed), intake)),
      new TurnToTarget(0, drive),
      new ParallelCommandGroup(
          new RunCommand(() -> hopper.setHopperTime(3), hopper),
          new RunCommand(() -> shoot.setSpeed(.5), shoot)
    ));
  }
}
