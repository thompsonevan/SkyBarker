package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;

public class Gripper {
    VictorSPX gripper;

    public Gripper(int CANID) {
        gripper = new VictorSPX(CANID);

        gripper.setNeutralMode(NeutralMode.Brake);

        gripper.setInverted(true);
    }
    

    public void action(RobotCommander commander) {
        gripper.set(ControlMode.PercentOutput, commander.getGripperCommand());
        SmartDashboard.putNumber("Gripper Command", commander.getGripperCommand());
    }
}
