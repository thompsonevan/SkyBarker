package frc.robot.subsystems;

import org.hotutilites.hotlogger.HotLogger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Elbow {
    private VictorSPX elbow;
    private CANCoder elbowEncoder;
    private double elbowAngle;

    public Elbow (int motorCANID, int encoderCANID) {
        
        elbow = new VictorSPX(motorCANID);
        elbow.configFactoryDefault();
        elbowEncoder = new CANCoder(encoderCANID);
        elbowEncoder.configFactoryDefault();
        elbowEncoder.setPositionToAbsolute();
        elbowEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        elbowEncoder.configMagnetOffset(Constants.ELBOW_OFFSET);

        //Configure sensor source for primary PID
        elbow.configRemoteFeedbackFilter(elbowEncoder, Constants.ELBOW_K_PID_LOOP_IDX);
        
        elbow.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, Constants.ELBOW_K_PID_LOOP_IDX,
        200);   

        
        elbow.setSensorPhase(true);
        elbow.setInverted(false);
        
        /* Set relevant frame periods to be at least as fast as periodic rate */
        // elbow.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ARM_TIMEOUT);
        // elbow.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,10);
        
        /* Set the peak and nominal outputs */
        elbow.configNominalOutputForward(0, Constants.ARM_TIMEOUT);
        elbow.configNominalOutputReverse(0, Constants.ARM_TIMEOUT);
        elbow.configPeakOutputForward(1, Constants.ARM_TIMEOUT);
        elbow.configPeakOutputReverse(-1, Constants.ARM_TIMEOUT);
        
        /* Set Motion Magic gains in slot0 - see documentation */
        elbow.selectProfileSlot(Constants.ELBOW_PID_SLOT, Constants.ELBOW_K_PID_LOOP_IDX);
        elbow.config_kF(Constants.ELBOW_PID_SLOT, Constants.ELBOW_MOTOR_kF, Constants.ARM_TIMEOUT);
        elbow.config_kP(Constants.ELBOW_PID_SLOT, Constants.ELBOW_MOTOR_kP, Constants.ARM_TIMEOUT);
        elbow.config_kI(Constants.ELBOW_PID_SLOT, Constants.ELBOW_MOTOR_kI, Constants.ARM_TIMEOUT);
        elbow.config_kD(Constants.ELBOW_PID_SLOT, Constants.ELBOW_MOTOR_kD, Constants.ARM_TIMEOUT);
        
        /* Set acceleration and vcruise velocity - see documentation */
        elbow.configMotionCruiseVelocity(Constants.ELBOW_CRUIESVELOCITY, Constants.ARM_TIMEOUT);
        elbow.configMotionAcceleration(Constants.ELBOW_ACCEL, Constants.ARM_TIMEOUT);

        // elbow.setSelectedSensorPosition(elbowEncoder.getPosition()*4096/360);

        elbow.configSensorTerm(SensorTerm.valueOf(314*4096/360), FeedbackDevice.RemoteSensor0);
    }

    private double convertToTicks(double inches) {
        return inches * Constants.FALCON500_TICKS_PER_REV * Constants.EXTENSION_RATIO;
    }
    
    private double convertToInches(double ticks) {
        return ticks / Constants.FALCON500_TICKS_PER_REV / Constants.EXTENSION_RATIO;
    }

    public void updatePose() {
        elbowAngle = elbow.getSelectedSensorPosition()*360/4096; 
        SmartDashboard.putNumber("Elbow Angle", elbowAngle);
        SmartDashboard.putNumber("Elbow Angle CANCODER", elbowEncoder.getAbsolutePosition());
        HotLogger.Log("Extension Pos",elbowAngle);
    }

    public void setMotorCommand(double motorCommand) {
        if (Math.abs(motorCommand) > .15) {
            elbow.set(ControlMode.PercentOutput, motorCommand);
        } else {
            elbow.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void goToPostion(double degrees) {
        elbow.set(ControlMode.MotionMagic, degrees);
    }

    public void setCoastMode() {
        elbow.setNeutralMode(NeutralMode.Coast);
    }

    public void setBreakMode() {
        elbow.setNeutralMode(NeutralMode.Brake);
    }
}
