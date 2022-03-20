// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public final WPI_TalonSRX m_leftMaster;
  public final WPI_TalonSRX m_leftSlave;
  public final WPI_TalonSRX m_rightMaster;
  public final WPI_TalonSRX m_rightSlave;

  private final MotorControllerGroup m_left;
  private final MotorControllerGroup m_right;

  private final DifferentialDrive m_diffDrive;

  private final DutyCycleEncoder m_rightEncoder;
  private final DutyCycleEncoder m_leftEncoder;

  private final ADXRS450_Gyro m_gyro;

  private final DifferentialDriveOdometry m_odometry;

  private final Pose2d m_pose;
  public final Field2d m_field;

  private final Timer m_timer;

  

  public Drivetrain() {
  m_leftMaster =  new WPI_TalonSRX(DriveConstants.leftMaster);
  m_leftSlave = new WPI_TalonSRX(DriveConstants.leftSlave);
  m_rightMaster = new WPI_TalonSRX(DriveConstants.rightMaster);
  m_rightSlave = new WPI_TalonSRX(DriveConstants.rightSlave);

    
    m_leftMaster.configOpenloopRamp(.05);
    m_leftSlave.configOpenloopRamp(.05);
    m_rightMaster.configOpenloopRamp(.05);
    m_rightSlave.configOpenloopRamp(.05);
    

/*
  m_rightMaster.setInverted(true);
  m_rightSlave.setInverted(true);
  m_leftMaster.setInverted(false);
  m_leftSlave.setInverted(false);
*/

  m_rightMaster.setNeutralMode(NeutralMode.Brake);
  m_rightSlave.setNeutralMode(NeutralMode.Brake);
  m_leftMaster.setNeutralMode(NeutralMode.Brake);
  m_rightSlave.setNeutralMode(NeutralMode.Brake);
    
  m_left = new MotorControllerGroup(m_leftMaster, m_leftSlave);
  m_right = new MotorControllerGroup(m_rightMaster, m_rightSlave);
  m_left.setInverted(true);
  //m_right.setInverted(true);

  m_diffDrive = new DifferentialDrive(m_left, m_right);
  m_diffDrive.setDeadband(DriveConstants.deadband);

  m_rightEncoder = new DutyCycleEncoder(DriveConstants.rightEncoderPort);
  m_leftEncoder = new DutyCycleEncoder(DriveConstants.leftEncoderPort);
  
  m_rightEncoder.setDistancePerRotation(DriveConstants.gearRatio * DriveConstants.wheelDiameter * (Math.PI));
  m_leftEncoder.setDistancePerRotation(DriveConstants.gearRatio * DriveConstants.wheelDiameter * (Math.PI));

    //Should default to CS0 port
  m_gyro = new ADXRS450_Gyro();

  m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

  m_pose = new Pose2d();
  m_field = new Field2d();
  //m_field.setRobotPose(xMeters, yMeters, rotation);
  SmartDashboard.putData("Field", m_field);

  m_timer = new Timer();

  
  }


public void arcadeDrive(double move, double steer) {
  SmartDashboard.putNumber("arcadeMove", move * DriveConstants.driveSpeed);
  SmartDashboard.putNumber("arcadeSteer", steer * DriveConstants.driveSpeed);
  m_diffDrive.arcadeDrive((move) * DriveConstants.driveSpeed , (steer) * DriveConstants.driveSpeed);
  //m_diffDrive.feed();
}
public void tankDrive(double left, double right) {
  SmartDashboard.putNumber("tankLeft", left * DriveConstants.driveSpeed);
  SmartDashboard.putNumber("tankRight", right * DriveConstants.driveSpeed);
  m_diffDrive.tankDrive(left * DriveConstants.driveSpeed, right * DriveConstants.driveSpeed);
  //m_diffDrive.feed();
}
public void tankDriveVolts(double left, double right) {
  SmartDashboard.putNumber("tankVoltLeft", left * DriveConstants.driveSpeed);
  SmartDashboard.putNumber("tankVoltRight", right * DriveConstants.driveSpeed);
  m_left.setVoltage(MathUtil.clamp(left, -12, 12));
  m_right.setVoltage(MathUtil.clamp(right, -12, 12));
  m_diffDrive.feed();
}
public void curvatureDrive(double move, double steer, boolean rotate) {
  SmartDashboard.putNumber("curveMove", move * DriveConstants.driveSpeed);
  SmartDashboard.putNumber("curveSteer", steer * DriveConstants.driveSpeed);
  SmartDashboard.putBoolean("curveBool", rotate);
  m_diffDrive.curvatureDrive(move * DriveConstants.driveSpeed, steer * DriveConstants.driveSpeed, rotate);
  //m_diffDrive.feed();
}
public void driveTime(double seconds, double speed) {
 m_timer.start();
 while(m_timer.get() < seconds)
 arcadeDrive(speed,0);
 /*
  if (m_timer.getFPGATimestamp() < seconds) {
    arcadeDrive(speed,0);
  } else {
    m_timer.stop();
  }
  */
  m_timer.reset();
  m_diffDrive.stopMotor();
}
public void stopDriveMotors() {
  m_leftMaster.stopMotor();
  m_leftSlave.stopMotor();
  m_rightMaster.stopMotor();
  m_rightSlave.stopMotor();
}
public void setMaxOutput(double maxOutput) {
  m_diffDrive.setMaxOutput(maxOutput);
}
public double getRightEncoderPosition() {
  return m_rightEncoder.getDistance();
}
public double getLeftEncoderPosition() {
  return m_leftEncoder.getDistance();
}
public double getRightEncoderRate() {
  return m_rightEncoder.getDistancePerRotation();
}
public double getLeftEncoderRate() {
  return m_leftEncoder.getDistancePerRotation();
}
public double getAverageEncoderRate() {
  return (double) ((getLeftEncoderRate() + getRightEncoderRate()) / 2.0);
}
public double getAverageEncoderDistance() {
  return (double) ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
}
public double getAngleMethod() {
  return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
}
public void zeroHeading() {
  System.out.println("Gyro Reset");
  m_gyro.reset();
}
public double getTurnRate() {
  return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
}
public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate()); // we are sending in meters/second
}
public double getHeading() {
  return Math.IEEEremainder(getAngleMethod(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
}
public Pose2d getPose() {
  return m_odometry.getPoseMeters();
}
public void resetAllSensors() {
  m_rightEncoder.reset();
  m_leftEncoder.reset();
  zeroHeading();
  m_odometry.resetPosition(m_odometry.getPoseMeters(), m_gyro.getRotation2d());
  System.out.println("All Sensors Reset");
}
public void resetOdometry(Pose2d pose) {
  m_odometry.resetPosition(pose, m_gyro.getRotation2d());
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_odometry.update(
      m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
      m_field.setRobotPose(m_odometry.getPoseMeters());
  }
}
