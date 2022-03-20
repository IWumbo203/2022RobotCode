// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hopper;

public class ShooterWPIPID extends PIDSubsystem {

  /** Creates a new Shooter. */
  private CANSparkMax m_master = new CANSparkMax(ShooterConstants.masterPort, MotorType.kBrushless);
  private CANSparkMax m_slave = new CANSparkMax(ShooterConstants.slavePort, MotorType.kBrushless);

  private SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(
    ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

  private final Timer m_timer = new Timer();
  private final Hopper hopper;
  private BangBangController controller = new BangBangController();

  public ShooterWPIPID(Hopper hopper) {

    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    //Controller Dependencies
    getController().setTolerance(ShooterConstants.posTol);
    setSetpoint(ShooterConstants.kShooterTargetRPS);
    this.hopper = hopper;

    m_master.setIdleMode(IdleMode.kCoast);
    m_slave.setIdleMode(IdleMode.kCoast);
    //m_slave.setInverted(true);
    //m_slave.follow(m_master);
  }

  public void setSpeed(double speed) {
    m_master.set(speed);
    m_slave.set(-speed);
    SmartDashboard.putNumber("Shooter Encoder", m_master.getEncoder().getPosition());
  }
/*
  public void speedSet() {
    m_master.set(ShooterConstants.shootSpeed);
  }
*/
  public void autoShoot() {
    setSpeed(ShooterConstants.highSpeed);
    if (getRPM() >= 5000) {
    hopper.setHopper(HopperConstants.hopperSpeed);
    }
  }
  public void stopShootMotors() {
    m_master.stopMotor();
    m_slave.stopMotor();
  }
  public void useBangBang(double setpoint) {
  m_master.setVoltage(MathUtil.clamp(controller.calculate(getRPM(), setpoint), -12, 12));
  }
  public void useBangBangF(double setpoint) {
    m_master.setVoltage(MathUtil.clamp(controller.calculate(getRPM(), setpoint) * .6 * m_shooterFeedforward.calculate(setpoint), -12, 12));
    }
    
  @Override
  public void useOutput(double output, double setpoint) {
   m_master.setVoltage(MathUtil.clamp(output + m_shooterFeedforward.calculate(setpoint), -12, 12)); 
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_master.getEncoder().getVelocity();
  }
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
  
  public double getRPM() {
    return m_master.getEncoder().getVelocity();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", this.getMeasurement());
  }
  
}
