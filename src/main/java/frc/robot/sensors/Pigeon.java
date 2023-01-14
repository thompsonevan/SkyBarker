package frc.robot.sensors;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import org.hotutilites.hotlogger.HotLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutonCommader;
import frc.robot.RobotCommander;

import static frc.robot.Constants.*;

public class Pigeon{
    private static double theta;

    private static Pigeon2 pigeon;

    public Pigeon() {
        pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);
    }

    public static void zeroSensor() {
        pigeon.setYaw(0);
    }

    public static void zeroSensor(double angle) {
        pigeon.setYaw(angle);
    }

    public static double getAngle(){
        return pigeon.getYaw();
    }

    public static Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getAngle());
    }

    public static void logData() {
        SmartDashboard.putNumber("Theta", getAngle());

        HotLogger.Log("Theta", getAngle());
    }

    public void enabledAction(RobotCommander commander) {
        if (commander.getResetIMU()) {
            pigeon.setYaw(0);
        }
    }
}
