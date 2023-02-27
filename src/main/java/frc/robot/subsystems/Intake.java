package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotCommander;
import frc.robot.TeleopCommander;

import static frc.robot.Constants.*;

public class Intake {
    double speed;
    double angle;
    TalonFX speedMotor;
    CANSparkMax angleMotor;
    CANCoder angleEncoder;
    PIDController pidController;

    public static double startPosition;

    public Intake(){
        angle = 45;
        speed = 0;
        speedMotor = new TalonFX(9);
        angleMotor = new CANSparkMax(10, MotorType.kBrushless);
        angleEncoder = new CANCoder(23);
        pidController = new PIDController(.017, 0.0015, 0.000075); //.0175, 0.001, 0.000075
        startPosition = angleEncoder.getAbsolutePosition();
        angleEncoder.configSensorDirection(true);
        angleEncoder.configMagnetOffset(INTAKE_OFFSET);
        // //Configure sensor source for primary PID
        // angleMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, SHOULDER_K_PID_LOOP_IDX,
        // ARM_TIMEOUT);   

        // angleMotor.setSensorPhase(false);
        // angleMotor.setInverted(false);

        // /* Set relevant frame periods to be at least as fast as periodic rate */
        // angleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, INTAKE_TIMEOUT);
        // angleMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,10);
        // /* Set the peak and nominal outputs */
        // angleMotor.configNominalOutputForward(0, ARM_TIMEOUT);
        // angleMotor.configNominalOutputReverse(0, ARM_TIMEOUT);
        // angleMotor.configPeakOutputForward(1, ARM_TIMEOUT);
        // angleMotor.configPeakOutputReverse(-1, ARM_TIMEOUT);

        // /* Set Motion Magic gains in slot0 - see documentation */
        // angleMotor.selectProfileSlot(SHOULDER_PID_SLOT, SHOULDER_K_PID_LOOP_IDX);
        // angleMotor.config_kF(SHOULDER_PID_SLOT, SHOULDER_MOTOR_kF, ARM_TIMEOUT);
        // angleMotor.config_kP(SHOULDER_PID_SLOT, SHOULDER_MOTOR_kP, ARM_TIMEOUT);
        // angleMotor.config_kI(SHOULDER_PID_SLOT, SHOULDER_MOTOR_kI, ARM_TIMEOUT);
        // angleMotor.config_kD(SHOULDER_PID_SLOT, SHOULDER_MOTOR_kD, ARM_TIMEOUT);

        // /* Set acceleration and vcruise velocity - see documentation */
        // angleMotor.configMotionCruiseVelocity(SHOULDER_CRUISEVELOCITY, ARM_TIMEOUT);
        // angleMotor.configMotionAcceleration(SHOULDER_ACCEL, ARM_TIMEOUT);

        // /* Zero the sensor once on robot boot up */
        // angleMotor.setSelectedSensorPosition(0, SHOULDER_K_PID_LOOP_IDX, ARM_TIMEOUT);

    }

    public void setBrakeMode(){
        angleMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode(){
        angleMotor.setIdleMode(IdleMode.kCoast);
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
        if(commander.getIntakePosition()[0] == 10000){
            angleMotor.set(0);
        } else {
            angle = commander.getIntakePosition()[0];
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

    public void logData(){
        SmartDashboard.putNumber("Intake Absolute Encoder", angleEncoder.getAbsolutePosition());
    }
}
