package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;
import frc.robot.TeleopCommander;

import static frc.robot.Constants.*;

public class Intake {
    public static enum IntakePos {
        none(0),
        pack(90),
        downCube(185.5),
        downCone(185.5),
        armMoving(102),
        cubeHandoff(100);

        private final double pos;
        public double getPos() {
            return pos;
        }

        IntakePos(double pos) {
            this.pos = pos;
        }
    }

    public static enum IntakeSpeed {
        none(0,0),
        onCube(-.5,-.5),
        onCone(-.5,.5),
        out(.5,5),
        cubeHandoff(-1,-1);

        private final double sp1;
        public double getSpeed1() {
            return sp1;
        }

        private final double sp2;
        public double getSpeed2() {
            return sp2;
        }

        IntakeSpeed(double sp1, double sp2) {
            this.sp1 = sp1;
            this.sp2 = sp2;
        }
    }

    double speed1;
    double speed2;
    double angle;
    TalonFX speedMotor1;
    TalonFX speedMotor2;
    CANSparkMax angleMotor;
    CANCoder angleEncoder;
    PIDController pidController;
    public static double angleEncoderAngle;

    DigitalInput sensor;

    public static double startPosition;

    public Intake(){
        angle = 45;
        speed1 = 0;
        speed2 = 0;
        speedMotor1 = new TalonFX(INTAKE_SPEED1_MOTOR_ID);
        speedMotor2 = new TalonFX(INTAKE_SPEED2_MOTOR_ID);
        angleMotor = new CANSparkMax(INTAKE_ANGLE_MOTOR_ID, MotorType.kBrushless);
        angleEncoder = new CANCoder(23);
        pidController = new PIDController(.0175, 0.0015, 0.000075); //.0175, 0.001, 0.000075
        startPosition = angleEncoder.getAbsolutePosition();
        angleEncoder.configSensorDirection(true);
        angleEncoder.configMagnetOffset(INTAKE_OFFSET);
        sensor = new DigitalInput(0);

        speedMotor2.setInverted(TalonFXInvertType.OpposeMaster);
        speedMotor2.follow(speedMotor1);
    }

    public void setBrakeMode(){
        angleMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode(){
        angleMotor.setIdleMode(IdleMode.kCoast);
    }

    public void IntakePeriodic(RobotCommander commander){
        SmartDashboard.putNumber("Intake Desired Angle", commander.getIntakePosition().getPos());
        // SmartDashboard.putNumber("Intake Desired Speed", commander.getIntakePosition().getSpeed());
        speedPeriodic(commander);
        anglePeriodic(commander);
        SmartDashboard.putString("INtake Pos", commander.getIntakePosition().toString());

    }
    
    public void speedPeriodic(RobotCommander commander){
        IntakePos intakePos = commander.getIntakePosition();

        speed1 = commander.getIntakeSpeed().sp1;
        speed2 = commander.getIntakeSpeed().sp2;

        if (intakePos == IntakePos.cubeHandoff){
            if(Math.abs(intakePos.pos - angleEncoder.getAbsolutePosition()) < 3){
                speedMotor1.set(TalonFXControlMode.PercentOutput, speed1);
                speedMotor2.set(TalonFXControlMode.PercentOutput, speed2);
            } else {
                speedMotor1.set(TalonFXControlMode.PercentOutput, 0);
                speedMotor2.set(TalonFXControlMode.PercentOutput, 0);
            }
        } else {
            if(sensor.get()){
                speedMotor1.set(TalonFXControlMode.PercentOutput, 0);
                speedMotor2.set(TalonFXControlMode.PercentOutput, 0);
            } else {
                speedMotor1.set(TalonFXControlMode.PercentOutput, speed1);
                speedMotor2.set(TalonFXControlMode.PercentOutput, speed2);
            }
        }


    }

    public void anglePeriodic(RobotCommander commander){
        if(commander.getIntakePosition() == IntakePos.none){
            angleMotor.set(0);
        } else {
            angle = commander.getIntakePosition().getPos();
            double ourAngle = angleEncoder.getAbsolutePosition();
            double change = pidController.calculate(ourAngle, angle);
            SmartDashboard.putNumber("Change", change);
            if(Math.abs(ourAngle - angle) > .2){
                angleMotor.set(change);
            } else {
                angleMotor.set(0);
            }
        }

    }

    public void logData() {
        angleEncoderAngle = angleEncoder.getAbsolutePosition();
        SmartDashboard.putNumber("Intake Absolute Encoder", angleEncoderAngle);
        SmartDashboard.putBoolean("Intake Sensor Reading", sensor.get());
    }
}