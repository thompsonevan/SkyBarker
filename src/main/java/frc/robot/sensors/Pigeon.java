package frc.robot.sensors;

import com.ctre.phoenix.sensors.Pigeon2;
import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;

import static frc.robot.Constants.*;

public class Pigeon{
    private static Pigeon2 pigeon;
    private static Alliance alliance;

    public Pigeon() {
        if(realBot){
            pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID, "drivetrain");
        } else {
            pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);
        }
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

    public static double getPitch(){
        return pigeon.getPitch();
    }

    public static double getRoll(){
        return pigeon.getRoll();
    }

    public static Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(MathUtil.inputModulus(getAngle(), -180, 180));
    }

    public void logData() {
        SmartDashboard.putNumber("Theta", getAngle());
        SmartDashboard.putNumber("Pitch", getPitch());
        SmartDashboard.putNumber("Roll", getRoll());

        HotLogger.Log("Theta", getAngle());
    }
    
    public void enabledAction(RobotCommander commander) {
        if (commander.getResetIMU()) {
            pigeon.setYaw(0);
        }
    }

    public static int getScoringDirection()  {
        if (getAngle() > -170 && getAngle() > 0 ) {
            return 1;
        } else {
            return -1;
        }
    }
}
