package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm;

public abstract class RobotCommander {
    double[] intakeArray = {Constants.INTAKE_PACKAGE_POSITION, 0};

    public XboxController driver;
    public XboxController operator;

    public abstract double getForwardCommand();
    public abstract double getStrafeCommand();
    public abstract double getTurnCommand();
    public abstract boolean getResetIMU();
    public abstract double armShoulder();
    public abstract double armExtension();
    public abstract Arm.ArmPos getArmPosition();
    public abstract double[] getIntakePosition();
    public abstract boolean getPickUpObject();
    public abstract boolean getArmReset();
}
