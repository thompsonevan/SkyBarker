package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    static NetworkTable table;
    static NetworkTableEntry tx;
    static NetworkTableEntry ty;
    static NetworkTableEntry ta;
    static NetworkTableEntry tv;
    static NetworkTableEntry tclass;
    static NetworkTableEntry botpose;
    // static Pose2d botPose2d;

    public Camera(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        tclass = table.getEntry("tclass");
        // botpose = table.getEntry("botpose");
    }

    public static void switchPipe(boolean aprilTag){
        if(aprilTag){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        } else {
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
        }
    }

    // public static Pose2d getBotPose(){
    //     double[] defaultArray = {0,0,0,0,0,0};
    //     double[] aprilPose = botpose.getDoubleArray(defaultArray);
    //     botPose2d = new Pose2d(aprilPose[0], aprilPose[1], Rotation2d.fromDegrees(aprilPose[5]));
    //     return botPose2d;
    // }

    public static double getX(){
        return tx.getDouble(0);
    }

    public static double getY(){
        return ty.getDouble(0);
    }

    public static double getA(){
        return ta.getDouble(0);
    }

    public static double getV(){
        return tv.getDouble(0);
    }

    public static String getC(){
        return tclass.getString("none");
    }

    public void logData(){
        SmartDashboard.putNumber("_Object X", getX());
        SmartDashboard.putNumber("_Object Y", getY());
        SmartDashboard.putNumber("_Object V", getV());
        SmartDashboard.putNumber("_Object A", getA());
        SmartDashboard.putString("_Object C", getC());
        // SmartDashboard.putNumber("_Object Pose X", botPose2d.getX());
        // SmartDashboard.putNumber("_Object Pose Y", botPose2d.getY());
        // SmartDashboard.putNumber("_Object Pose Theta", botPose2d.getRotation().getDegrees());
    }
}
