// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends PIDSubsystem {
  /** Creates a new Shooter. */
  private CANSparkMax m_master = new CANSparkMax(ShooterConstants.masterPort, MotorType.kBrushless);
  private CANSparkMax m_slave = new CANSparkMax(ShooterConstants.slavePort, MotorType.kBrushless);
  private SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(
    ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);
  private final Timer m_timer = new Timer();
  public Shooter() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    //Controller Dependencies
    getController().setTolerance(ShooterConstants.posTol);
    setSetpoint(ShooterConstants.kShooterTargetRPS);
    m_slave.follow(m_master);
  }

  public void setSpeed(double speed) {
    m_master.set(speed);
    SmartDashboard.putNumber("Shooter Encoder", m_master.getEncoder().getPosition());
  }

  public void setShooterOnTimer(double seconds, double speed) {
    m_timer.reset();
    m_timer.start();

    while(m_timer.get() < seconds) {
      setSpeed(speed);
    }
    m_timer.stop();
  }

  @Override
  public void useOutput(double output, double setpoint) {
   m_master.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_master.getEncoder().getVelocity();
  }
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
