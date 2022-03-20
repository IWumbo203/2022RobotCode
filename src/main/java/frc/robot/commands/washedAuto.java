// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterWPIPID;

public class washedAuto extends CommandBase {
  /** Creates a new washedAuto. */
  private double time;
  private double seconds;
  private Drivetrain drive;
  private double driveSpeed;
  private ShooterWPIPID shoot;
  private Hopper hopper;
  private double shootSpeed;
  public washedAuto(Drivetrain drive, ShooterWPIPID shoot, Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    //time = 5;
    seconds = 3;
    driveSpeed = .5;
    shootSpeed = .5;
    this.drive = drive;
    this.shoot = shoot;
    this.hopper = hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() > 12) {
      new autoDrive(drive, 1, driveSpeed);
    } else if (Timer.getFPGATimestamp() < 12 && Timer.getFPGATimestamp() > 9) {
      new autoShoot(shoot, hopper, 5 , shootSpeed);
    } else if (Timer.getFPGATimestamp() < 9  && Timer.getFPGATimestamp() > 4) {
      new autoDrive(drive, 5 , -driveSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() <= 4;
  }
}
