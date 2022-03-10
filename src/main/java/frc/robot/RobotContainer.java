// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.DriveSetDistance;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.SwitchDriveMode;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PathWeaver;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain m_drivetrain = new Drivetrain();
  public final XboxController m_driverController = new XboxController(XboxConstants.port);
  public final Intake m_intake = new Intake();
  public final Hopper m_hopper = new Hopper();
  public final Shooter m_shooter = new Shooter();
  public final Command m_pathweaver = getTrajectoryCommand("FarLeft");
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Integer> m_numChooser = new SendableChooser<>();
  ArrayList<PathWeaver> trajectories = new ArrayList<PathWeaver>();
  //PathWeaver m_farLeft = new PathWeaver("FarLeft", 7.154, 5.575, new Rotation2d());
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    trajectories.add(0, new PathWeaver("FarLeft", 7.154, 5.575, new Rotation2d(-0.791, 1.306)));
    trajectories.add(1, new PathWeaver("MidLeft", 6.351, 3.982, new Rotation2d(-2.036, -.563)));
    trajectories.add(2, new PathWeaver("FarRight", 8.52, 1.850, new Rotation2d(0.611, -1.497)));
    trajectories.add(3, new PathWeaver("MidRight", 7.046, 2.904, new Rotation2d(-1.893, -0.874)));
    // Configure the button bindings
    configureButtonBindings();

    m_numChooser.setDefaultOption("TrajectoriesNum", 1);
    m_chooser.setDefaultOption("PathWeaver", getTrajectoryCommand(trajectories.get(m_numChooser.getSelected()).getPath()));
    
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    Shuffleboard.getTab("Autonomous").add(m_numChooser);

    Command m_teleopCommand = 
        new InstantCommand(m_shooter::enable, m_shooter)
        //.andThen(new RunCommand(() -> m_intake.setIntake(.2), m_intake))
        .andThen(new RunCommand(() -> m_hopper.setHopper()));

    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> 
      m_drivetrain.arcadeDrive(-m_driverController.getLeftY(), m_driverController.getRightX()), m_drivetrain));

  /* TEST CODE FOR TANK AND CURVATURE DRIVES
    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> 
      m_drivetrain.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY())));
      
    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> 
      m_drivetrain.tankDriveVolts(-m_driverController.getLeftY() * 12, -m_driverController.getRightY() * 12)));

    m_drivetrain.setDefaultCommand(
      new RunCommand(() -> 
      m_drivetrain.curvatureDrive(-m_driverController.getLeftY(), m_driverController.getRightX(), m_driverController.getAButton())));
      */
      }
    
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whenPressed(new SwitchDriveMode(m_drivetrain));

    new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new TurnToTarget(0, m_drivetrain).withTimeout(1));

    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new DriveToDistance(100, m_drivetrain).withTimeout(3)); 

    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(new DriveSetDistance(100, m_drivetrain).withTimeout(3));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whenPressed(new RunCommand(() -> m_intake.setIntake(), m_intake))
        .whenReleased(new RunCommand(() -> m_intake.setIntake(), m_intake));

    new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(
                new SequentialCommandGroup(
                new InstantCommand(m_shooter::enable, m_shooter),
                new ConditionalCommand(
                new InstantCommand(m_hopper::setHopper, m_hopper),
                new InstantCommand(),
                m_shooter::atSetpoint)))
        .whenReleased(
                new SequentialCommandGroup(
                new RunCommand(() -> m_hopper.setHopper()),
                new InstantCommand(m_shooter::disable, m_shooter)));
        
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

  public static Command getTrajectoryCommand(String path) {
    
    Trajectory exampleTrajectory = PathWeaver.getTrajectory(path);

    m_drivetrain.m_field.getObject("traj").setTrajectory(exampleTrajectory);
    
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_drivetrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.kinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrain::tankDriveVolts,
            m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
  
    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));

  }
}
