package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;
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
        speedMotor = new TalonFX(9);
        angleMotor = new CANSparkMax(10, MotorType.kBrushless);
        angleEncoder = new CANCoder(23);
        pidController = new PIDController(.01, 0, 0);
    }

    public void IntakePeriodic(RobotCommander commander){
        SmartDashboard.putNumber("Intake Desired Angle", commander.getIntakePosition()[0]);
        SmartDashboard.putNumber("Intake Desired Speed", commander.getIntakePosition()[1]);
        speedPeriodic(commander);
        anglePeriodic(commander);
    }

    public void speedPeriodic(RobotCommander commander){
        speed = commander.getIntakePosition()[1];
        SmartDashboard.putNumber("Speed", speed);
        speedMotor.set(TalonFXControlMode.PercentOutput, -speed);
    }

    public void anglePeriodic(RobotCommander commander){
        angle = commander.getIntakePosition()[0];
        double ourAngle = angleEncoder.getAbsolutePosition();
        double change = pidController.calculate(ourAngle, angle);
        SmartDashboard.putNumber("Change", change);
        if(Math.abs(ourAngle - angle) > 10){
            angleMotor.set(change);
        } else {
            angleMotor.set(0);
        }
    }

    public void logData(){
        SmartDashboard.putNumber("Intake Absolute Encoder", angleEncoder.getAbsolutePosition());
    }
}
