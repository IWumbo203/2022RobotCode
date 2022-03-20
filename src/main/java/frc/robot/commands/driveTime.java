// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class driveTime extends CommandBase {
  private final Drivetrain drive;
  private double startTime;
  private double time;
  private double seconds;
  private double driveSpeed;
  Timer timer = new Timer();
  /** Creates a new Drive. */
  public driveTime(Drivetrain drive, double seconds, double driveSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.seconds = seconds;
    this.driveSpeed = driveSpeed;
    addRequirements(drive);
    //startTime = 15;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if(Timer.getFPGATimestamp - time  < 3) //do first command
    //if(Timer.getFPGATimestamp - time  > 3 && < 5)
    //if(Timer.getFPGATimestamp - time  > 5 && < 10)
    if (timer.get() < seconds) {
      drive.arcadeDrive(driveSpeed, 0);
    } else {
      drive.stopDriveMotors();
  }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > seconds;
  }
}
