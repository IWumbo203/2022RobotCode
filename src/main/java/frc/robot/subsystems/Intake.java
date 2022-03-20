// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax m_intakeMotor;
  private final Timer m_timer;
  public Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.intakePort, MotorType.kBrushless);
    m_timer = new Timer();
    m_intakeMotor.setInverted(true);
  }

  public void setIntake(double speed) {
    m_intakeMotor.set(speed);
  }
  public void stopIntake() {
    m_intakeMotor.set(0);
  }
  public void setIntakeTimer(double seconds) {
    m_timer.reset();
    m_timer.start();

    while(m_timer.get() < seconds) {
      setIntake(IntakeConstants.intakeSpeed);
    }
    m_timer.stop();
    m_intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
