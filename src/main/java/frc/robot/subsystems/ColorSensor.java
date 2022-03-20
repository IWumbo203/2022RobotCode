// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  /** Creates a new ColorSensor. */
  private ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher;

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  public ColorSensor() {
m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
m_colorMatcher = new ColorMatch();
m_colorMatcher.addColorMatch(kBlueTarget);
m_colorMatcher.addColorMatch(kGreenTarget);
m_colorMatcher.addColorMatch(kRedTarget);
m_colorMatcher.addColorMatch(kYellowTarget);    
  }
public String readColor() {
  Color detectedColor = m_colorSensor.getColor();
  String colorString;
  ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    return colorString;
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
