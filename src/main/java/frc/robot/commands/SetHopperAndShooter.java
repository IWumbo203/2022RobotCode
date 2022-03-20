// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterWPIPID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetHopperAndShooter extends SequentialCommandGroup {
  /** Creates a new SetHopperAndShooter. */
  public SetHopperAndShooter(ShooterWPIPID m_shooter, Hopper m_hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      new InstantCommand(m_shooter::enable, m_shooter),
                new ConditionalCommand(
                new InstantCommand(m_hopper::startHopper, m_hopper),
                new InstantCommand(),
                m_shooter::atSetpoint),
                new SequentialCommandGroup(
                new RunCommand(() -> m_hopper.startHopper(), m_hopper),
                new InstantCommand(m_shooter::disable, m_shooter)));

  }
}

