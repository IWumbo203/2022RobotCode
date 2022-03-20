// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterWPIPID;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeprecatedHopAndShoot extends ParallelCommandGroup {
  /** Creates a new DeprecatedHopAndShoot. */
  public DeprecatedHopAndShoot(Hopper hopper, ShooterWPIPID shoot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> shoot.setSpeed(ShooterConstants.highSpeed), shoot),
      new WaitCommand(2),
      //new RunCommand(() -> Timer.delay(2)),
      new RunCommand(() -> hopper.setHopper(HopperConstants.hopperSpeed), hopper),
      new WaitCommand(2),
      new RunCommand(() -> shoot.stopShootMotors(), shoot),
      new RunCommand(() -> hopper.stopHopperMotor(), hopper)
    );
  }
}
