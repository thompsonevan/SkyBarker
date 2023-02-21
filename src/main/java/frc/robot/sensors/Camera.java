package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    static NetworkTable limelightLeft;
    static NetworkTable limelightRight;
    static NetworkTableEntry leftBotPose;
    static NetworkTableEntry rightBotPose;

    public Camera(){
        limelightLeft = NetworkTableInstance.getDefault().getTable("limelight-fluke");
        limelightRight = NetworkTableInstance.getDefault().getTable("limelight");
        leftBotPose = limelightLeft.getEntry("botpose");
        rightBotPose = limelightRight.getEntry("botpose");
    }

    public static Pose2d getLeftBotPose(){
        double[] defaultArray = {0,0,0,0,0,0};
        double[] aprilPose = leftBotPose.getDoubleArray(defaultArray);
        return new Pose2d(aprilPose[0], aprilPose[1], Rotation2d.fromDegrees(aprilPose[5]));
    }

    public static Pose2d getRightBotPose(){
        double[] defaultArray = {0,0,0,0,0,0};
        double[] aprilPose = rightBotPose.getDoubleArray(defaultArray);
        return new Pose2d(aprilPose[0], aprilPose[1], Rotation2d.fromDegrees(aprilPose[5]));
    }

    public void logData(){
        SmartDashboard.putNumber("Left Bot Pose X", getLeftBotPose().getX());
        SmartDashboard.putNumber("Left Bot Pose Y", getLeftBotPose().getY());
        SmartDashboard.putNumber("Left Bot Pose Theta", getLeftBotPose().getRotation().getDegrees());

        SmartDashboard.putNumber("Right Bot Pose X", getRightBotPose().getX());
        SmartDashboard.putNumber("Right Bot Pose Y", getRightBotPose().getY());
        SmartDashboard.putNumber("Right Bot Pose Theta", getRightBotPose().getRotation().getDegrees());
    }
}
