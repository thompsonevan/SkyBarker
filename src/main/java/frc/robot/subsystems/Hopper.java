package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;
import static frc.robot.Constants.*;

import org.hotutilites.hotlogger.HotLogger;

public class Hopper {
    public static int PixieCam = 0;
    public static Servo fingerServo = new Servo(FINGER_SERVO);
    public static DigitalInput sensorTop= new DigitalInput(TOP_SENSOR);
    public static DigitalInput sensorLeft= new DigitalInput(LEFT_SENSOR);
    public static DigitalInput sensorRight= new DigitalInput(RIGHT_SENSOR);
    public static DigitalInput sensorBottom= new DigitalInput(BOTTOM_SENSOR);
    public static CANSparkMax hopperMotor = new CANSparkMax(HOPPER_MOTOR, MotorType.kBrushless);
    public static HopperCam hopperCam = HopperCam.CUBE;
    public static boolean hopperOverrideActive = false;

    enum HopperCam {
      NONE,
      CUBE,
      CONE
    }

    public void HopperPeriodic(RobotCommander commander){
        sensorTop.get();
        sensorLeft.get();
        sensorRight.get();
        sensorBottom.get();

        if (commander.hopperOverrideLeft()) {
            hopperOverrideActive = true;
            hopperMotor.set(HOPPER_OVERRIDE_SPEED);

        } else if (commander.hopperOverrideRight()) {
            hopperOverrideActive = true;
            hopperMotor.set(-1 * HOPPER_OVERRIDE_SPEED);

        } else {
            hopperOverrideActive = false;
            if (hopperCam == HopperCam.CUBE) {
                if (sensorLeft.get() && sensorBottom.get()) {
                    hopperMotor.set(.25);
                    fingerServo.setAngle(0);
                } else if (sensorRight.get() && sensorBottom.get() ) {
                    hopperMotor.set(-1 * .25);
                    fingerServo.setAngle(0);
                } else {
                    hopperMotor.setIdleMode(IdleMode.kBrake);
                    hopperMotor.set(0);
                    fingerServo.setAngle(0);
                }
            }
            else if (hopperCam == HopperCam.CONE) {
                if (sensorLeft.get() && sensorBottom.get() && !sensorTop.get()) {
                    hopperMotor.set(.25);
                    fingerServo.setAngle(90);
                } else if (sensorRight.get() && sensorBottom.get() && !sensorTop.get()) {
                    hopperMotor.set(-1 * .25);
                    fingerServo.setAngle(90);
                } else {
                    hopperMotor.setIdleMode(IdleMode.kBrake);
                    hopperMotor.set(0);
                    fingerServo.setAngle(0);
                }
            } else {
                hopperMotor.setIdleMode(IdleMode.kBrake);
                hopperMotor.set(0);
                fingerServo.setAngle(0); 
            }
        }
    }

  public void logData(){
    SmartDashboard.putBoolean("HopSensor Bottom", sensorBottom.get());
    SmartDashboard.putBoolean("HopSensor Left", sensorLeft.get());
    SmartDashboard.putBoolean("HopSensor Right", sensorRight.get());
    SmartDashboard.putBoolean("HopSensor Top", sensorTop.get());
    SmartDashboard.putBoolean("Hopper Override", hopperOverrideActive);

    HotLogger.Log("HopSensor Bottom", sensorBottom.get());
    HotLogger.Log("HopSensor Left", sensorLeft.get());
    HotLogger.Log("HopSensor Right", sensorRight.get());
    HotLogger.Log("HopSensor Top", sensorTop.get());
    HotLogger.Log("Hopper Override", hopperOverrideActive);
    }
}
  
