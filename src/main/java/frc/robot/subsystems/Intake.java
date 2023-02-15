package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.TeleopCommander;

public class Intake {
    double speed;
    double angle;
    TalonFX speedMotor;
    CANSparkMax angleMotor;
    CANCoder angleEncoder;
    PIDController pidController;

    public Intake(){
        angle = 45;
        speed = 0;
        speedMotor = new TalonFX(90);
        angleMotor = new CANSparkMax(90, MotorType.kBrushless);
        angleEncoder = new CANCoder(20);
        pidController = new PIDController(.01, 0, 0);
    }

    public void IntakePeriodic(TeleopCommander commander){
        speedPeriodic(commander);
        anglePeriodic(commander);
    }

    public void speedPeriodic(TeleopCommander commander){
        speed = commander.getIntakePosition()[1];
        speedMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void anglePeriodic(TeleopCommander commander){
        angle = commander.getIntakePosition()[0];
        double ourAngle = angleEncoder.getAbsolutePosition();
        double change = pidController.calculate(ourAngle, angle);
        if(Math.abs(ourAngle - angle) > 5){
            angleMotor.set(change);
        }
    }
}
