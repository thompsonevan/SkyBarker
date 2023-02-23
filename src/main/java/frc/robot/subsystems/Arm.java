package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotCommander;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import static frc.robot.Constants.*;

import org.hotutilites.hotlogger.HotLogger;

public class Arm {
    public static enum ArmPos {
        topNode,
        middleNode,
        lowerNode,
        packagePos,
        manual,
        stay
    }

    TalonFX shoulder;
    CANCoder shoulderEncoder;

    double shoulderDesPos;
    double extensionDesPos;
    double elbowDesPos;
    private double shoulderAngleMotor;
    private double shoulderAngleCANCODER;

    private Extension extension;
    private Elbow elbow;

    public Arm(){

        extension = new Extension(Constants.EXTENSION);
        elbow = new Elbow(Constants.ELBOW, Constants.ELBOW_ENCODER);

        shoulder = new TalonFX(SHOULDER);
        shoulder.configFactoryDefault();
        shoulderEncoder = new CANCoder(SHOULDER_ENCODER);
        shoulderEncoder.configFactoryDefault();
        shoulderEncoder.setPositionToAbsolute();
        shoulderEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        shoulderEncoder.configMagnetOffset(Constants.SHOULDER_OFFSET);


        //Configure sensor source for primary PID
        shoulder.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, SHOULDER_K_PID_LOOP_IDX,
        ARM_TIMEOUT);   

        shoulder.setSensorPhase(false);
        shoulder.setInverted(false);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        shoulder.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ARM_TIMEOUT);
        shoulder.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,10);

        /* Set the peak and nominal outputs */
        shoulder.configNominalOutputForward(0, ARM_TIMEOUT);
        shoulder.configNominalOutputReverse(0, ARM_TIMEOUT);
        shoulder.configPeakOutputForward(1, ARM_TIMEOUT);
        shoulder.configPeakOutputReverse(-1, ARM_TIMEOUT);

        /* Set Motion Magic gains in slot0 - see documentation */
        shoulder.selectProfileSlot(SHOULDER_PID_SLOT, SHOULDER_K_PID_LOOP_IDX);
        shoulder.config_kF(SHOULDER_PID_SLOT, SHOULDER_MOTOR_kF, ARM_TIMEOUT);
        shoulder.config_kP(SHOULDER_PID_SLOT, SHOULDER_MOTOR_kP, ARM_TIMEOUT);
        shoulder.config_kI(SHOULDER_PID_SLOT, SHOULDER_MOTOR_kI, ARM_TIMEOUT);
        shoulder.config_kD(SHOULDER_PID_SLOT, SHOULDER_MOTOR_kD, ARM_TIMEOUT);

        /* Set acceleration and vcruise velocity - see documentation */
        shoulder.configMotionCruiseVelocity(SHOULDER_CRUISEVELOCITY, ARM_TIMEOUT);
        shoulder.configMotionAcceleration(SHOULDER_ACCEL, ARM_TIMEOUT);

        /* Zero the sensor once on robot boot up */
        shoulder.setSelectedSensorPosition(0, SHOULDER_K_PID_LOOP_IDX, ARM_TIMEOUT);
    }
    
    public void armPercentOutZero(){
        shoulder.set(ControlMode.PercentOutput, 0);
    }

    // public void armZeroSensorPos(){
    //     shoulder.setSelectedSensorPosition(shoulderEncoder.getAbsolutePosition() * SHOULDER_DEGREES_TO_TICKS);
    //     extension.setSelectedSensorPosition(0);
    //     elbow.setSelectedSensorPosition(0);//elbowEncoder.getAbsolutePosition() * ELBOW_DEGREES_TO_TICKS
    // }

    public void setPosition(double shoudlerPos, double extensionPos, double elbowPos){
        shoulder.set(ControlMode.MotionMagic, shoudlerPos);
    }

    public void action(RobotCommander commander) {
        if (commander.operator.getAButton()) {
            elbow.goToPostion(90.0);
        } else {
            elbow.setMotorCommand(commander.operator.getLeftY()*.8);
        }
    }


    // public void action(RobotCommander commander){
    //     if(commander.getArmReset()){
    //         armZeroSensorPos();
    //     }

    //     if(commander.getArmPosition() == ArmPos.packagePos){
    //         setPosition(SHOULDER_TARGET_POSITION_PACKAGE, EXTENSION_TARGET_POSITION_PACKAGE, ELBOW_TARGET_POSITION_PACKAGE);
    //         shoulderDesPos = SHOULDER_TARGET_POSITION_PACKAGE;
    //         extensionDesPos = EXTENSION_TARGET_POSITION_PACKAGE;
    //         elbowDesPos = ELBOW_TARGET_POSITION_PACKAGE;
    //     } else if (commander.getArmPosition() == ArmPos.lowerNode){
    //         setPosition(SHOULDER_TARGET_POSITION_LOW, EXTENSION_TARGET_POSITION_LOW, ELBOW_TARGET_POSITION_LOW);
    //         shoulderDesPos = SHOULDER_TARGET_POSITION_LOW;
    //         extensionDesPos = EXTENSION_TARGET_POSITION_LOW;
    //         elbowDesPos = ELBOW_TARGET_POSITION_LOW;
    //     } else if (commander.getArmPosition() == ArmPos.middleNode){
    //         setPosition(SHOULDER_TARGET_POSITION_MIDDLE, EXTENSION_TARGET_POSITION_MIDDLE, ELBOW_TARGET_POSITION_MIDDLE);
    //         shoulderDesPos = SHOULDER_TARGET_POSITION_MIDDLE;
    //         extensionDesPos = EXTENSION_TARGET_POSITION_MIDDLE;
    //         elbowDesPos = ELBOW_TARGET_POSITION_MIDDLE;
    //     } else if (commander.getArmPosition() == ArmPos.topNode){
    //         setPosition(SHOULDER_TARGET_POSITION_HIGH, EXTENSION_TARGET_POSITION_HIGH, ELBOW_TARGET_POSITION_HIGH);
    //         shoulderDesPos = SHOULDER_TARGET_POSITION_HIGH;
    //         extensionDesPos = EXTENSION_TARGET_POSITION_HIGH;
    //         elbowDesPos = ELBOW_TARGET_POSITION_HIGH;
    //     } else if (commander.getArmPosition() == ArmPos.stay){
    //         armPercentOutZero();
    //     } else if(commander.getArmPosition() == ArmPos.manual){
    //         if ((shoulder.getSelectedSensorPosition() / FALCON500_TICKS_PER_REV) > 170 && commander.armShoulder() > 0){
    //             shoulder.set(ControlMode.PercentOutput, 0);
    //         } else if((shoulder.getSelectedSensorPosition() / FALCON500_TICKS_PER_REV) < -170  && commander.armShoulder() < 0){
    //             shoulder.set(ControlMode.PercentOutput, 0);
    //         } else{
    //             shoulder.set(ControlMode.PercentOutput, commander.armShoulder());
    //         }

    //         if ((extension.getSelectedSensorPosition() / FALCON500_TICKS_PER_REV)> 70  && commander.armExtension() > 0){
    //             extension.set(ControlMode.PercentOutput, 0);
    //         } else if ((extension.getSelectedSensorPosition() / FALCON500_TICKS_PER_REV) < 5 && commander.armExtension() < 0){
    //             extension.set(ControlMode.PercentOutput, 0);
    //         } else{
    //             extension.set(ControlMode.PercentOutput, commander.armExtension());
    //         }

    //         if(Math.abs(commander.operator.getRightX()) > .2){
    //             elbow.set(ControlMode.PercentOutput, commander.operator.getRightX());
    //         } else {
    //             elbow.set(ControlMode.PercentOutput, 0);
    //         }

    //     }
    // }

    public void updatePose(){

        extension.updatePose();
        elbow.updatePose();

        shoulderAngleMotor = shoulder.getSelectedSensorPosition() / Constants.FALCON500_TICKS_PER_REV / Constants.SHOULDER_RATIO;
        shoulderAngleCANCODER = shoulderEncoder.getAbsolutePosition();

        SmartDashboard.putNumber("Shoulder Angle Motor", shoulderAngleMotor);
        SmartDashboard.putNumber("Shoulder Angle Motor With ratio", Constants.SHOULDER_RATIO);
        SmartDashboard.putNumber("Shoulder Angle Motor", shoulderAngleMotor);
        SmartDashboard.putNumber("Shoulder Angle CANCODER", shoulderAngleCANCODER);

        HotLogger.Log("Shoulder Absolute Pos", shoulderEncoder.getAbsolutePosition());
        HotLogger.Log("Shoulder Motor Pos", shoulderAngleMotor);
        HotLogger.Log("Shoulder Desired Pos", shoulderDesPos);
        HotLogger.Log("Extension Desired Pos", extensionDesPos);
        HotLogger.Log("Elbow Desired Pos", elbowDesPos);
    }

    public void brakeMode(){
        shoulder.setNeutralMode(NeutralMode.Brake);
        extension.setBreakMode();
        elbow.setBreakMode();
    }

    public void coastMode(){
        shoulder.setNeutralMode(NeutralMode.Coast);
        extension.setCoastMode();
        elbow.setCoastMode();
    }

}