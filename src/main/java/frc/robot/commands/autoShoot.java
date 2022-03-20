// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterWPIPID;

public class autoShoot extends CommandBase {
  /** Creates a new autoShoot. */
  private double startTime;
  private double time;
  private ShooterWPIPID shoot;
  private Hopper hopper;
  private double seconds;
  private double shootSpeed;
  public autoShoot(ShooterWPIPID shoot, Hopper hopper, double seconds, double shootSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoot = shoot;
    this.hopper = hopper;
    this.seconds = seconds;
    this.shootSpeed = shootSpeed;
    addRequirements(shoot);
    addRequirements(hopper);
    startTime = Timer.getFPGATimestamp();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - time < seconds) {
      shoot.setSpeed(shootSpeed);
      hopper.setHopper(-HopperConstants.hopperSpeed);
    } else {
      shoot.stopShootMotors();
      hopper.stopHopperMotor();
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - time > seconds;
  }
}
