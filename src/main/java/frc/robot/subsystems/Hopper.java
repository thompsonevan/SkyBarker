package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotCommander;

import static frc.robot.Constants.*;

public class Hopper {
    CANSparkMax hopper;

    public Hopper(){
        hopper = new CANSparkMax(HOPPER_MOTOR, MotorType.kBrushless);
    }

    public void enabled(RobotCommander commander){
        if(Math.abs(commander.operator.getLeftY()) > .2){
            hopper.set(commander.operator.getLeftY() * .6);
        }
    }
}
