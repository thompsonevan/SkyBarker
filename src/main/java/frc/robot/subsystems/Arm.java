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

import org.hotutilites.hotlogger.HotLogger;

public class Arm {
    public static enum ArmPos {
        packagePos(0,.1,0),
        readyPosition(-23,.1,100),
        topNode(50,20,181),
        middleNode(51,.2,181),
        lowerNode(27,.2,69),
        manual(0,0,0), // manual motor commands
        Zero(0,0,0), // No motor command
        humanPlayer(22,11,92),
        intake(0,10,0), 
        outOfHopperToDirection(-20,.5,5), 
        outOfDirectionToHopper1(-0,.5,80),
        outOfPostiveToHopper2(-20,.5,10),
        outOfHopperToMid(-20,.5,80) ;

        private final double shoulder;
        public double getShoulder() {
            return shoulder;
        }

        private final double extension;
        public double getExtension() {
            return extension;
        }

        private final double elbow;

        public double getElbow() {
            return elbow;
        }

        ArmPos(double shoulder, double extension, double elbow) {
            this.shoulder = shoulder;
            this.extension = extension;
            this.elbow = elbow;
        }

        
    }

    double shoulderDesPos;
    double extensionDesPos;
    double elbowDesPos;
    private double shoulderAngleMotor;
    private double shoulderAngleCANCODER;

    private Extension extension;
    private Elbow elbow;
    private Shoulder shoulder;
    private ArmPos armTargetPrevious;
    private ArmZone currentZone;
    private ArmZone currentCommandedZone = ArmZone.none;
    private ArmPos actualCommand;
    private boolean achivedPostion;
    private boolean transitionStateInProgress;

    private boolean useNegativeSide;

    public Arm(){
        shoulder = new Shoulder(Constants.SHOULDER, Constants.SHOULDER_ENCODER);
        extension = new Extension(Constants.EXTENSION);
        elbow = new Elbow(Constants.ELBOW, Constants.ELBOW_ENCODER);
    }

    public void initilizeOffsets() {
        shoulder.intilizeOffset();
        elbow.intilizeOffset();

        currentCommandedZone = ArmZone.hopper;
    }
    
    public void armPercentOutZero(){
        shoulder.setMotorCommand(0.0);
        elbow.setMotorCommand(0.0);
        extension.setMotorCommand(0.0);
    }

    public enum ArmZone {
        hopper,
        negative,
        postive, anyZone, none
    }

    public ArmZone determineArmZone(double shoulder, double extension, double elbow) {

        if (elbow < -40 || (shoulder > 17 && shoulder < 22 && elbow < 10)) {
            return ArmZone.negative;
        } else if (elbow > 40 || (shoulder < -17 && shoulder > -22  && elbow > -10)) {
            return ArmZone.postive;
        } else {
            return ArmZone.hopper;
        }
    }

    public void action(RobotCommander commander) {
        if(Math.abs(elbow.getElbowAngle()) < 10){
            useNegativeSide = commander.useNegativeSide();
        }

        if (commander.getArmPosition() == ArmPos.manual) {
            actualCommand = ArmPos.manual;
            transitionStateInProgress = false;
        } else if (commander.getArmPosition() == ArmPos.Zero) {
            elbow.setMotorCommand(0.0);
            extension.setMotorCommand(0.0);
            shoulder.setMotorCommand(0.0);
            actualCommand = ArmPos.Zero;
            transitionStateInProgress = false;
        } else if (commander.getArmPosition() != armTargetPrevious) {
            if (useNegativeSide) {
                currentCommandedZone = this.determineArmZone(-commander.getArmPosition().getShoulder(), 
                                                             commander.getArmPosition().getExtension(), 
                                                             -commander.getArmPosition().getElbow());
            } else {
                currentCommandedZone = this.determineArmZone(commander.getArmPosition().getShoulder(), 
                                                             commander.getArmPosition().getExtension(), 
                                                             commander.getArmPosition().getElbow());
            }

            if (currentCommandedZone == currentZone) {
                actualCommand = commander.getArmPosition();
                transitionStateInProgress = false;
            } else if (currentCommandedZone == ArmZone.postive && currentZone == ArmZone.hopper) {
                actualCommand = ArmPos.outOfHopperToDirection;
                transitionStateInProgress = true;
            } else if (currentCommandedZone == ArmZone.hopper && currentZone == ArmZone.postive) {
                actualCommand = ArmPos.outOfDirectionToHopper1;
                transitionStateInProgress = true;
            } else if (currentCommandedZone == ArmZone.negative && currentZone == ArmZone.hopper) {
                actualCommand = ArmPos.outOfHopperToDirection;
                transitionStateInProgress = true;
            } else if (currentCommandedZone == ArmZone.hopper && currentZone == ArmZone.negative) {
                actualCommand = ArmPos.outOfDirectionToHopper1;
                transitionStateInProgress = true;
            }
        } else if (commander.getArmPosition() == armTargetPrevious ) {
            if (transitionStateInProgress) {
                if (achivedPostion) {
                    if (actualCommand == ArmPos.outOfDirectionToHopper1) {
                        actualCommand = ArmPos.outOfPostiveToHopper2;
                        transitionStateInProgress = true;
                    } else if (actualCommand == ArmPos.outOfHopperToDirection && 
                               (commander.getArmPosition() == ArmPos.middleNode || commander.getArmPosition() == ArmPos.topNode)) {
                                actualCommand = ArmPos.outOfHopperToMid;
                                transitionStateInProgress = true;
                    } else {
                        actualCommand = commander.getArmPosition();
                        transitionStateInProgress = true;
                    }
                }
            }
        }
        SmartDashboard.putString("Commanded Position", commander.getArmPosition().name());
        SmartDashboard.putString("Commanded Position Actual", actualCommand.name());
        SmartDashboard.putString("Commanded Zone", currentCommandedZone.name());
        SmartDashboard.putString("Commanded Position Previuos", commander.getArmPosition().name());
        armTargetPrevious = commander.getArmPosition();
        
        if (actualCommand != ArmPos.Zero && actualCommand != ArmPos.manual) {
            if (useNegativeSide) {
                shoulder.goToPostion(-actualCommand.getShoulder());
                extension.goToPostion(actualCommand.getExtension());
                elbow.goToPostion(-actualCommand.getElbow());
            } else {
                shoulder.goToPostion(actualCommand.getShoulder());
                extension.goToPostion(actualCommand.getExtension());
                elbow.goToPostion(actualCommand.getElbow());
            }
        } else if (actualCommand == ArmPos.manual) {
            elbow.setMotorCommand(commander.armElbow());
            extension.setMotorCommand(commander.armExtension());
            shoulder.setMotorCommand(commander.armShoulder());
        } else {
            shoulder.setMotorCommand(0);
            elbow.setMotorCommand(0);
            extension.setMotorCommand(0);
        }
    }

    public void updatePose(){
        extension.updatePose();
        elbow.updatePose();
        shoulder.updatePose();
        achivedPostion = this.determineAchivePosition();
        currentZone = this.determineArmZone(shoulder.getShoulderAngle(), extension.getExtensionPosition(), elbow.getElbowAngle());
        SmartDashboard.putString("ArmActualZone",currentZone.name());
        SmartDashboard.putBoolean("Achived Position",achivedPostion);
    }

    private boolean determineAchivePosition() {
        return shoulder.getAchivedTarget() && extension.getAchivedTarget() && elbow.getAchivedTarget();
    }

    public void brakeMode(){
        shoulder.setBreakMode();
        extension.setBreakMode();
        elbow.setBreakMode();
    }

    public void coastMode(){
        shoulder.setCoastMode();;
        extension.setCoastMode();
        elbow.setCoastMode();
    }

}