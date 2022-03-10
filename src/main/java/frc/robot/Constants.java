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

        public static final double driveSpeed = .6;

        public static final double deadband = .05;

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
    }
    public final class LimelightConstants {
        //MAKE SURE THIS IS IN METERS
        public static final double targetHeight = 0;
        public static final double limelightHeight = 0;
        public static final double mountAngle = 0;
    }
    public final class TurnToTargetConstants {
        public static final double kP = 0;
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
        public static final int hopperPort = 1;
        public static final double hopperSpeed = .2;
    }
    public static final class IntakeConstants {
        public static final int intakePort = 0;
        public static final double intakeSpeed = 0;
    }
    public static final class ShooterConstants {
        public static final int masterPort = 2;
        public static final int slavePort = 3;

        public static final double kSVolts = 0;
        public static final double kVVoltSecondsPerRotation = 0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double posTol = 0;
        public static final double veloTol = 0;

        public static final double kShooterTargetRPS = 0;

        public static final double kEncoderDistancePerPulse = 0;
    }
}


