package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Camera2 {

    private NetworkTable table;
    private NetworkTableEntry botpose;
    private double[] defaultPostion = {0,0,0,0,0,0};
    private NetworkTableEntry tagDetected;

    public Camera2(String name) {
        table = NetworkTableInstance.getDefault().getTable(name);
        tagDetected = table.getEntry("tv");
        if (DriverStation.getAlliance() == Alliance.Red) {
            botpose = table.getEntry("botpose_wpiblue");
            System.out.println("Here");
        } else {
            botpose = table.getEntry("botpose_wpired");
            System.out.println("There");
        }
    }

    public Pose2d getBotPose(){
        double[] rawPose = botpose.getDoubleArray(defaultPostion);
        return new Pose2d(rawPose[0], rawPose[1], new Rotation2d(Math.toRadians(rawPose[5])));
    }

    public boolean aprilTagsDetected() {
        if (tagDetected.getDouble(0.0) >= 0.9)
            return true;
        else
            return false;
    }
}
