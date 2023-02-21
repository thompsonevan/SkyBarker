package frc.robot.sensors;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon {
    Pigeon2 pigeon;
    private Rotation2d yaw;
    public Rotation2d getYaw() {
        return yaw;
    }

    public Pigeon(int canID) {
        pigeon = new Pigeon2(canID);
        this.updatePose();
    }

    public Pigeon(int canID, String canBus) {
        pigeon = new Pigeon2(canID,canBus);
        this.updatePose();
    }

    public Rotation2d getRotation2d() {
        return yaw;
    }

    public Rotation2d getRotation2dWrapped() {
        return Rotation2d.fromRadians(MathUtil.angleModulus(yaw.getRadians()));
    }

    public void setYaw(double degrees) {
        pigeon.setYaw(degrees);
    }

    public void updatePose() {
        yaw = Rotation2d.fromDegrees(pigeon.getYaw());

        SmartDashboard.putNumber("Raw Pigeon Yaw", yaw.getDegrees());
        SmartDashboard.putNumber("Wrapped Pigeon Yaw", Rotation2d.fromRadians(MathUtil.angleModulus(yaw.getRadians())).getDegrees());
    }
}
