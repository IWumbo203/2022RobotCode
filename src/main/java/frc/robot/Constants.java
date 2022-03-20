// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class DriveConstants {
        public static final int rightMaster = 3;
        public static final int rightSlave = 4;
        public static final int leftSlave = 2;
        public static final int leftMaster = 1;

        public static final double driveSpeed = .9;

        public static final double deadband = .1;

        //public static final int rightEncoderPortA = 0;
        //public static final int rightEncoderPortB = 0;
        public static final int rightEncoderPort = 0;
        //public static final int leftEncoderPortA = 0;
        //public static final int leftEncoderPortB = 0;
        public static final int leftEncoderPort = 1;
        //meters please
        public static final double wheelDiameter = .1524;
        public static final double gearRatio = 1;
        
        public static final double trackWidth = 0.5842;

        public static final boolean kGyroReversed = false;

        public static final double ksVolts = 0;
        public static final double kvVoltSecondsPerMeter = 0;
        public static final double kaVoltSecondsSquaredPerMeter = 0;

        public static final double kPDriveVel = 0;

        //public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);
    }
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);
    
    public final class AutoConstants {
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = .7;
    }
    public final class XboxConstants {
        public static final int port = 0;
        public static final int port2 = 1;
    }
    public final class LimelightConstants {
        //MAKE SURE THIS IS IN METERS
        public static final double targetHeight = 2.7178;
        public static final double limelightHeight = 0.4826;
        public static final double mountAngle = 45;
    }
    public final class TurnToTargetConstants {
        public static final double kP = .05;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double posTol = 0;
        public static final double veloTol = 0;
    }
    public final class DriveToDistanceConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double posTol = 0;
        public static final double veloTol = 0;
    }
    public final class DriveSetDistanceConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double posTol = 0;
        public static final double veloTol = 0;
    }
    public final class TurnDegreesPIDConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double posTol = 0;
        public static final double veloTol = 0;
    }
    public static final class HopperConstants {
        public static final int hopperPort = 5;
        public static final double hopperSpeed = 1;
    }
    public static final class IntakeConstants {
        public static final int intakePort = 8;
        public static final double intakeSpeed = .5;
        public static final double reverseIntakeSpeed = -.5;
    }
    public static final class ShooterConstants {
        public static final int masterPort = 3;
        public static final int slavePort = 1; 


        //FOR SPARKMAXPIDCONTROLLER
        public static final int smartMotionSlot = 0;
        public static final double maxVel = 2000;
        public static final double minVel = 0;
        public static final double maxAcc = 1500;
        
        public static final double allErr = 0;
        public static final double maxRPM = 5700;
        public static final double minRPM = 0;

        public static final double iZone = 0;
        public static final double ff = 0;

        public static final double lowSpeed = .25;
        public static final double highSpeed = .425;

        public static final double kSVolts = 0.22092;
        public static final double kVVoltSecondsPerRotation = 0.062875;
        
        public static final double kP = 0.46584;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kPV = 8.4506E-14;
        public static final double kIV = 0;
        public static final double kDV = 0;

        public static final double posTol = 0;
        public static final double veloTol = 0;

        public static final double kShooterTargetRPS = 4400;

        public static final double kEncoderDistancePerPulse = 0;
    }
}


