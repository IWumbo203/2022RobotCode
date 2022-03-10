// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
public class SwitchDriveMode extends CommandBase {
  /** Creates a new SwitchDriveMode. */
  private final Drivetrain m_drivetrain;
  //String mode;
  boolean bool;
  public SwitchDriveMode(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = drive;
    //this.mode = mode;
    bool = true;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bool = !bool;
    //mode.equals("Brake")
    if (bool == true) {
      m_drivetrain.m_rightMaster.setNeutralMode(NeutralMode.Brake);
      m_drivetrain.m_rightSlave.setNeutralMode(NeutralMode.Brake);
      m_drivetrain.m_leftMaster.setNeutralMode(NeutralMode.Brake);
      m_drivetrain.m_leftSlave.setNeutralMode(NeutralMode.Brake);
      System.out.println("Mode: Brake");
    } else {
      m_drivetrain.m_rightMaster.setNeutralMode(NeutralMode.Coast);
      m_drivetrain.m_rightSlave.setNeutralMode(NeutralMode.Coast);
      m_drivetrain.m_leftMaster.setNeutralMode(NeutralMode.Coast);
      m_drivetrain.m_leftSlave.setNeutralMode(NeutralMode.Coast);
      System.out.println("Mode: Coast");
    }
 
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
