// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  CANSparkMax m_hopperMotor;
  private final Timer m_timer;
  public Hopper() {
    m_hopperMotor = new CANSparkMax(HopperConstants.hopperPort, MotorType.kBrushless);
    m_timer = new Timer();
  }

  public void setHopper() {
    m_hopperMotor.set(HopperConstants.hopperSpeed);
  }
  public void stopHopper() {
    m_hopperMotor.set(0);
  }
  public void setHopperTime(double seconds) {
    m_timer.reset();
    m_timer.start();

    while(m_timer.get() < seconds) {
      setHopper();
    }
    m_timer.stop();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
