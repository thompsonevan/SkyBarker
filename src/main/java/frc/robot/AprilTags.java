package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class AprilTags {
    public static double iToM(double inches) {
        return inches / 39.37;
    }

    static AprilTag april1 = new AprilTag(1, new Pose3d(
            iToM(610.77),
            iToM(42.19),
            iToM(18.22),
            new Rotation3d(0, 0, Math.toRadians(180))));
    static AprilTag april2 = new AprilTag(2, new Pose3d(
            iToM(610.77),
            iToM(108.19),
            iToM(18.22),
            new Rotation3d(0, 0, Math.toRadians(180))));
    static AprilTag april3 = new AprilTag(3, new Pose3d(
            iToM(610.77),
            iToM(147.19),
            iToM(18.22),
            new Rotation3d(0, 0, Math.toRadians(180))));
    static AprilTag april4 = new AprilTag(4, new Pose3d(
            iToM(636.96),
            iToM(265.74),
            iToM(27.38),
            new Rotation3d(0, 0, Math.toRadians(180))));
    static AprilTag april5 = new AprilTag(5, new Pose3d(
            iToM(14.25),
            iToM(265.74),
            iToM(27.38),
            new Rotation3d(0, 0, Math.toRadians(0))));
    static AprilTag april6 = new AprilTag(6, new Pose3d(
            iToM(40.45),
            iToM(147.19),
            iToM(18.22),
            new Rotation3d(0, 0, Math.toRadians(0))));
    static AprilTag april7 = new AprilTag(7, new Pose3d(
            iToM(40.45),
            iToM(108.19),
            iToM(18.22),
            new Rotation3d(0, 0, Math.toRadians(0))));
    static AprilTag april8 = new AprilTag(8, new Pose3d(
            iToM(40.45),
            iToM(42.19),
            iToM(18.22),
            new Rotation3d(0, 0, Math.toRadians(0))));

    static List<AprilTag> aprils = new ArrayList<AprilTag>(){{
        add(april1);
        add(april2);
        add(april3);
        add(april4);
        add(april5);
        add(april6);
        add(april7);
        add(april8);
    }};

    static AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(aprils, 16.54, 8.02);

    public void a(){
        int tagID = 1;
        
        if(tagID < 1){
            tagID = 1;
        } else if (tagID > 8){
            tagID = 8;
        }

        Pose3d test = fieldLayout.getTagPose(tagID).get();
    }
}
