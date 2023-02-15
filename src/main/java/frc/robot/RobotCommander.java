package frc.robot;

import frc.robot.subsystems.Arm;

public abstract class RobotCommander {
    double[] intakeArray = {0, 0};

    public abstract double getForwardCommand();
    public abstract double getStrafeCommand();
    public abstract double getTurnCommand();
    public abstract boolean getResetIMU();
    public abstract boolean getArmPosition1();
    public abstract boolean getArmPosition2();
    public abstract boolean getArmPosition3();
    public abstract boolean getArmPositionPackage();
    public abstract double armShoulder();
    public abstract double armExtension();
    public abstract Arm.ArmPos getArmPosition();
    public abstract double[] getIntakePosition();

}
