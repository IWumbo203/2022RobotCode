// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSPARKPID extends SubsystemBase {
  /** Creates a new ShooterSPARKPID. */
  private CANSparkMax m_master = new CANSparkMax(ShooterConstants.masterPort, MotorType.kBrushless);
  private CANSparkMax m_slave = new CANSparkMax(ShooterConstants.slavePort, MotorType.kBrushless);

  private SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(
    ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

  private SparkMaxPIDController m_pidController;

  private final Hopper hopper;
  

  public ShooterSPARKPID(Hopper hopper) {
    m_pidController = m_master.getPIDController();
    m_pidController.setP(ShooterConstants.kPV);
    m_pidController.setI(ShooterConstants.kIV);
    m_pidController.setD(ShooterConstants.kDV);
    m_pidController.setIZone(ShooterConstants.iZone);
    m_pidController.setFF(ShooterConstants.ff);
    m_pidController.setOutputRange(-1, 1);

    m_pidController.setSmartMotionMaxVelocity(ShooterConstants.maxVel, ShooterConstants.smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(ShooterConstants.minVel, ShooterConstants.smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(ShooterConstants.maxAcc, ShooterConstants.smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(ShooterConstants.allErr, ShooterConstants.smartMotionSlot);

    m_pidController.setReference(ShooterConstants.kShooterTargetRPS, CANSparkMax.ControlType.kVelocity);

  
    this.hopper = hopper;

    m_master.setIdleMode(IdleMode.kCoast);
    m_slave.setIdleMode(IdleMode.kCoast);
    m_slave.setInverted(true);
    m_slave.follow(m_master);
  }

  public void setSpeed(double speed) {
    m_master.set(speed);
   // m_slave.set(-speed);
    SmartDashboard.putNumber("Shooter Encoder", m_master.getEncoder().getPosition());
  }
/*
  public void speedSet() {
    m_master.set(ShooterConstants.shootSpeed);
  }
*/
  public void autoShoot() {
    m_master.set(ShooterConstants.highSpeed);
    if (getRPM() >= 5000) {
    hopper.setHopper(HopperConstants.hopperSpeed);
    }
  }
  public void stopShootMotors() {
    m_master.stopMotor();
    m_slave.stopMotor();
  }
  
  public double getRPM() {
    return m_master.getEncoder().getVelocity();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}

