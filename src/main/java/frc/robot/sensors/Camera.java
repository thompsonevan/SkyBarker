package frc.robot.sensors;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    static NetworkTable limelightLeft;
    static NetworkTable limelightRight;
    static NetworkTableEntry leftBotPose;
    static NetworkTableEntry rightBotPose;

    static NetworkTableEntry leftTagDetected;
    static NetworkTableEntry rightTagDetected;

    static Pose2d leftPose = new Pose2d(0,0, new Rotation2d());
    static Pose2d rightPose = new Pose2d(0,0, new Rotation2d());

    public Camera(){
        limelightLeft = NetworkTableInstance.getDefault().getTable("limelight-chkfan");
        limelightRight = NetworkTableInstance.getDefault().getTable("limelight");
        leftBotPose = limelightLeft.getEntry("botpose");
        rightBotPose = limelightRight.getEntry("botpose");

        leftTagDetected = limelightLeft.getEntry("tv");
        rightTagDetected = limelightRight.getEntry("tv");

        Alliance color = Alliance.Blue;

        if (color == Alliance.Blue) {
            leftBotPose = limelightLeft.getEntry("botpose_wpiblue");
            rightBotPose = limelightRight.getEntry("botpose_wpiblue");
            System.out.println("Here");
        } else {
            leftBotPose = limelightLeft.getEntry("botpose_wpired");
            rightBotPose = limelightRight.getEntry("botpose_wpired");
            System.out.println("There");
        }
    }

    public static Pose2d getLeftBotPose(){
        double[] defaultArray = {0,0,0,0,0,0};
        double[] aprilPose = leftBotPose.getDoubleArray(defaultArray);
        rightPose = new Pose2d(aprilPose[0], aprilPose[1], new Rotation2d(Math.toRadians(aprilPose[5])));
        return rightPose;
    }

    public static Pose2d getRightBotPose(){
        double[] defaultArray = {0,0,0,0,0,0};
        double[] aprilPose = rightBotPose.getDoubleArray(defaultArray);
        leftPose = new Pose2d(aprilPose[0], aprilPose[1], new Rotation2d(Math.toRadians(aprilPose[5])));
        return leftPose;
    }

    public static boolean leftAprilDetected() {
        if (leftTagDetected.getDouble(0.0) >= 0.9)
            return true;
        else
            return false;
    }

    public static boolean rightAprilDetected() {
        if (rightTagDetected.getDouble(0.0) >= 0.9)
            return true;
        else
            return false;
    }

    public void logData(){
        SmartDashboard.putNumber("Left Bot Pose X", getLeftBotPose().getX());
        SmartDashboard.putNumber("Left Bot Pose Y", getLeftBotPose().getY());
        SmartDashboard.putNumber("Left Bot Pose Theta", getLeftBotPose().getRotation().getDegrees());
        // SmartDashboard.putNumber("Left Target Seen", limelightLeft.getEntry('tv'));

        SmartDashboard.putNumber("Right Bot Pose X", getRightBotPose().getX());
        SmartDashboard.putNumber("Right Bot Pose Y", getRightBotPose().getY());
        SmartDashboard.putNumber("Right Bot Pose Theta", getRightBotPose().getRotation().getDegrees());
    }
}
