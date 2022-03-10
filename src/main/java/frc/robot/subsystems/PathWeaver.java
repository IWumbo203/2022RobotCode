// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class PathWeaver {
    private  double xMeters;
    private  double yMeters;
    private Rotation2d rotation2d;
    private String path;
        public PathWeaver(String path, double xMeters, double yMeters, Rotation2d rotation2d) {
            this.path = path;
            this.xMeters = xMeters;
            this.yMeters = yMeters;
            this.rotation2d = rotation2d;
        }
        public String getPath() {
            return path;
        }
        public double getXMeters() {
            return xMeters;
        }
        public double getYMeters() {
            return yMeters;
        }
        public Rotation2d getRotation2d() {
            return rotation2d;
        }
        public void setPath(String newPath) {
            path = newPath;
        }
        public void setXMeters(double newX) {
            xMeters = newX;
        }
        public void setYMeters(double newY) {
            yMeters = newY;
        }
        public void setRotation2d(Rotation2d newRotation2d) {
            rotation2d = newRotation2d;
        }
    public static Trajectory getTrajectory(String path) {
        String realPath = "output/" + path + ".wpilib.json";
        //DriveToTarget.wpilib.json
        Trajectory newTrajectory = new Trajectory();
        try {
            //"/home/lvuser/deploy/output/paths/" + path + ".wpilib.json"
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(realPath);
          // File trajectoryPath = new File(Filesystem.getDeployDirectory(), "paths/" + path + ".wpilib");
           newTrajectory =  TrajectoryUtil.fromPathweaverJson(trajectoryPath);
           return newTrajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
            return null;
        }
    }
}
