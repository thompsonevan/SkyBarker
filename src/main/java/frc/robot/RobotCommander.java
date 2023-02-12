package frc.robot;

public abstract class RobotCommander {
    public abstract double getForwardCommand();
    public abstract double getStrafeCommand();
    public abstract double getTurnCommand();
    public abstract boolean getResetIMU();
    public abstract boolean getArmPosition1();
    public abstract boolean getArmPosition2();
}
