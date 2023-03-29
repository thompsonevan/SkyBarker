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
    public static NetworkTable limelightLeft;
    public static NetworkTable limelightRight;
    static NetworkTableEntry leftBotPose;
    static NetworkTableEntry rightBotPose;
    private static double[] defaultPos = {0,0,0,0,0,0};

    static NetworkTableEntry captureLatencyLeft;
    static NetworkTableEntry targetingLatencyLeft;
    static NetworkTableEntry captureLatencyRight;
    static NetworkTableEntry targetingLatencyRight;

    static NetworkTableEntry leftTagDetected;
    static NetworkTableEntry rightTagDetected;

    static Pose2d leftPose = new Pose2d(0,0, new Rotation2d());
    static Pose2d rightPose = new Pose2d(0,0, new Rotation2d());

    public Camera(){
        limelightLeft = NetworkTableInstance.getDefault().getTable("limelight-left");
        limelightRight = NetworkTableInstance.getDefault().getTable("limelight-right");

        leftBotPose = limelightLeft.getEntry("botpose_wpired");
        rightBotPose = limelightRight.getEntry("botpose_wpired");

        leftTagDetected = limelightLeft.getEntry("tv");
        rightTagDetected = limelightRight.getEntry("tv");

        captureLatencyLeft = limelightLeft.getEntry("cl");
        targetingLatencyLeft = limelightLeft.getEntry("tl");
        captureLatencyRight = limelightRight.getEntry("cl");
        targetingLatencyRight = limelightRight.getEntry("tl");
    }

    public void enabled(){
        limelightLeft.getEntry("ledMode").setNumber(3);
        limelightRight.getEntry("ledMode").setNumber(3);
    }

    public void disabled(){
        limelightLeft.getEntry("ledMode").setNumber(1);
        limelightRight.getEntry("ledMode").setNumber(1);
    }

    public static Pose2d getLeftBotPose(){
        double[] rawPose = leftBotPose.getDoubleArray(defaultPos);
        return new Pose2d(rawPose[0], rawPose[1], new Rotation2d(Math.toRadians(rawPose[5])));
    }

    public static double getCaptureLatencyRight(){
        return captureLatencyRight.getDouble(0);
    }

    public static double getTargetingLatencyRight(){
        return targetingLatencyRight.getDouble(0);
    }

    public static double getCaptureLatencyLeft(){
        return captureLatencyLeft.getDouble(0);
    }

    public static double getTargetingLatencyLeft(){
        return targetingLatencyLeft.getDouble(0);
    }

    public static Pose2d getRightBotPose(){
        double[] rawPose = rightBotPose.getDoubleArray(defaultPos);
        return new Pose2d(rawPose[0], rawPose[1], new Rotation2d(Math.toRadians(rawPose[5])));
    }

    public static boolean getLeftDetecting(){
        return leftTagDetected.getDouble(0) == 1;
    }

    public static boolean getRightDetecting(){
        return rightTagDetected.getDouble(0) == 1;
    }


    public static boolean rightAprilDetected() {
        return false;
    }
}
