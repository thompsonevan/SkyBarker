package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Arm.ArmPos;

import static frc.robot.Constants.*;


public class TeleopCommander extends RobotCommander{

    // public XboxController driver;
    // public  XboxController operator;
    private static boolean Bumpercheck = false;
    
    private boolean manualMode = false;

    private boolean yButtonPressed = false;
    private boolean aButtonPressed = false;
    private boolean xButtonPressed = false;
    private boolean bButtonPressed = false;

    private boolean cubeMode = true; // true ball, false cone

    public TeleopCommander() {
        driver = new XboxController(0);
        operator = new XboxController(1);
    }

    @Override
    public double getForwardCommand() {
        return (modifyAxis(driver.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND);
    }

    @Override
    public double getStrafeCommand() {
        return (modifyAxis(driver.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND);
    }

    @Override
    public double getTurnCommand() {
        double value = deadband(Math.abs(driver.getRightX()) * driver.getRightX(), 0.13, 0.4) * (MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

        return value;
    }

    @Override
    public boolean getResetIMU() {
        return driver.getBackButton();
    }

    public boolean getDriveToScoring(){
        return driver.getBButton();
    }

    private double deadband(double value, double deadband, double maxRange){
        if(Math.abs(value) < deadband){
            return 0;
        } else if (value < 0) {
            return  ((value + deadband)/(1.0 - deadband)) * maxRange;
        } else {
            return  ((value - deadband)/(1.0 - deadband)) * maxRange;
        }
    }
  
    private double modifyAxis(double value) {
        boolean deadband = 0.13 > Math.sqrt(Math.pow(driver.getLeftX(), 2) + Math.pow(driver.getLeftY(), 2));

        if (deadband) {
            return 0;
        } else {
            return Math.abs(value) * value;
        }
    }
    
    public double[] getIntakePosition() {
        boolean Dpad_right = (operator.getPOV() > 70 && operator.getPOV() < 110);
        boolean Dpad_left = (operator.getPOV() > 250 && operator.getPOV() < 290);
        boolean Dpad_updown = ((operator.getPOV() > 160 && operator.getPOV() < 200) || (!(operator.getPOV() < 0) && operator.getPOV() < 20));
        boolean Trigger_right = (operator.getRightTriggerAxis() > .3);
        boolean Trigger_left = (operator.getLeftTriggerAxis() > .3);
        boolean Bumper_push = operator.getRightBumperPressed();
        boolean Bumper_release = operator.getRightBumperReleased();

        if(getCubeMode()){
            if (Trigger_left && !Trigger_right) {
                if (Dpad_left && !(Dpad_right || Dpad_updown)) {
                    intakeArray[0] = Constants.INTAKE_PACKAGE_POSITION;
                    intakeArray[1] = Constants.INTAKE_SPEED_CUBE;
                } else if (Dpad_right && !(Dpad_left || Dpad_updown)) {
                    intakeArray[0] = Constants.INTAKE_COLLECT_POSITION;
                    intakeArray[1] = Constants.INTAKE_SPEED_CUBE;
                } else if (Dpad_updown && !(Dpad_left || Dpad_right)) {
                    intakeArray[0] = Constants.INTAKE_STATION_POSITION;
                    intakeArray[1] = Constants.INTAKE_SPEED_CUBE;
                } else {
                    intakeArray[1] = Constants.INTAKE_SPEED_CUBE;
                }
            } else if (Trigger_right && !Trigger_left) {
                if (Dpad_left && !(Dpad_right || Dpad_updown)) {
                    intakeArray[0] = Constants.INTAKE_PACKAGE_POSITION;
                    intakeArray[1] = -Constants.INTAKE_SPEED_CUBE;
                } else if (Dpad_right && !(Dpad_left || Dpad_updown)) {
                    intakeArray[0] = Constants.INTAKE_COLLECT_POSITION;
                    intakeArray[1] = -Constants.INTAKE_SPEED_CUBE;
                } else if (Dpad_updown && !(Dpad_left || Dpad_right)) {
                    intakeArray[0] = Constants.INTAKE_STATION_POSITION;
                    intakeArray[1] = -Constants.INTAKE_SPEED_CUBE;
                } else {
                    intakeArray[1] = -Constants.INTAKE_SPEED_CUBE;
                }
            } else if (Bumper_push && !(Dpad_left || Dpad_right || Dpad_updown)) {
                Bumpercheck = true;
                intakeArray[0] = Constants.INTAKE_COLLECT_POSITION;
                intakeArray[1] = 0;
            } else if (Bumper_release && !(Dpad_left || Dpad_right || Dpad_updown) && Bumpercheck) {
                Bumpercheck = false;
                intakeArray[0] = Constants.INTAKE_PACKAGE_POSITION;
                intakeArray[1] = 0;
            } else if (Dpad_left && !(Trigger_right || Trigger_left || Dpad_right || Dpad_updown)) {
                intakeArray[0] = Constants.INTAKE_PACKAGE_POSITION;
                intakeArray[1] = .0;
            } else if (Dpad_right && !(Trigger_left || Trigger_right || Dpad_left || Dpad_updown)) {
                intakeArray[0] = Constants.INTAKE_COLLECT_POSITION;
                intakeArray[1] = .0;
            } else if (Dpad_updown && !(Trigger_left || Trigger_right || Dpad_left || Dpad_right)) {
                intakeArray[0] = Constants.INTAKE_STATION_POSITION;
                intakeArray[1] = .0;
            } else {
                intakeArray[1] = .0;
            }
        } else {
            if (Trigger_left && !Trigger_right) {
                if (Dpad_left && !(Dpad_right || Dpad_updown)) {
                    intakeArray[0] = Constants.INTAKE_PACKAGE_POSITION;
                    intakeArray[1] = Constants.INTAKE_SPEED_CONE;
                } else if (Dpad_right && !(Dpad_left || Dpad_updown)) {
                    intakeArray[0] = Constants.INTAKE_COLLECT_POSITION;
                    intakeArray[1] = Constants.INTAKE_SPEED_CONE;
                } else if (Dpad_updown && !(Dpad_left || Dpad_right)) {
                    intakeArray[0] = Constants.INTAKE_STATION_POSITION;
                    intakeArray[1] = Constants.INTAKE_SPEED_CONE;
                } else {
                    intakeArray[1] = Constants.INTAKE_SPEED_CONE;
                }
            } else if (Trigger_right && !Trigger_left) {
                if (Dpad_left && !(Dpad_right || Dpad_updown)) {
                    intakeArray[0] = Constants.INTAKE_PACKAGE_POSITION;
                    intakeArray[1] = -Constants.INTAKE_SPEED_CONE;
                } else if (Dpad_right && !(Dpad_left || Dpad_updown)) {
                    intakeArray[0] = Constants.INTAKE_COLLECT_POSITION;
                    intakeArray[1] = -Constants.INTAKE_SPEED_CONE;
                } else if (Dpad_updown && !(Dpad_left || Dpad_right)) {
                    intakeArray[0] = Constants.INTAKE_STATION_POSITION;
                    intakeArray[1] = -Constants.INTAKE_SPEED_CONE;
                } else {
                    intakeArray[1] = -Constants.INTAKE_SPEED_CONE;
                }
            } else if (Bumper_push && !(Dpad_left || Dpad_right || Dpad_updown)) {
                Bumpercheck = true;
                intakeArray[0] = Constants.INTAKE_COLLECT_POSITION;
                intakeArray[1] = 0;
            } else if (Bumper_release && !(Dpad_left || Dpad_right || Dpad_updown) && Bumpercheck) {
                Bumpercheck = false;
                intakeArray[0] = Constants.INTAKE_PACKAGE_POSITION;
                intakeArray[1] = 0;
            } else if (Dpad_left && !(Trigger_right || Trigger_left || Dpad_right || Dpad_updown)) {
                intakeArray[0] = Constants.INTAKE_PACKAGE_POSITION;
                intakeArray[1] = .0;
            } else if (Dpad_right && !(Trigger_left || Trigger_right || Dpad_left || Dpad_updown)) {
                intakeArray[0] = Constants.INTAKE_COLLECT_POSITION;
                intakeArray[1] = .0;
            } else if (Dpad_updown && !(Trigger_left || Trigger_right || Dpad_left || Dpad_right)) {
                intakeArray[0] = Constants.INTAKE_STATION_POSITION;
                intakeArray[1] = .0;
            } else {
                intakeArray[1] = .0;
            }
        }
        
        return intakeArray;
    }

    public boolean getArmReset(){
        return operator.getBackButton();
    }

    public boolean getManualMode(){
        if(!manualMode && operator.getStartButtonPressed()){
            manualMode = true;
        } else if (manualMode && operator.getStartButtonPressed()){
            manualMode = false;
        }

        return manualMode;
    }

    public ArmPos getArmPosition(){
        if(operator.getYButton()){
            yButtonPressed = true;
            aButtonPressed = false;
            bButtonPressed = false;
            xButtonPressed = false;
        } else if(operator.getAButton()){
            yButtonPressed = false;
            aButtonPressed = true;
            bButtonPressed = false;
            xButtonPressed = false;
        } else if(operator.getBButton()){
            yButtonPressed = false;
            aButtonPressed = false;
            bButtonPressed = true;
            xButtonPressed = false;
        } else if(operator.getXButton()){
            yButtonPressed = false;
            aButtonPressed = false;
            bButtonPressed = false;
            xButtonPressed = true;
        }

        if (getManualMode()){
            yButtonPressed = false;
            aButtonPressed = false;
            bButtonPressed = false;
            xButtonPressed = false;
            return ArmPos.manual;
        } else {
            if(yButtonPressed){
                return ArmPos.topNode;
            } else if (aButtonPressed){
                return ArmPos.packagePos;
            } else if (bButtonPressed){
                return ArmPos.middleNode;
            } else if (xButtonPressed){
                return ArmPos.lowerNode;
            } else {
                return ArmPos.stay;
            }
        }
    }

    public boolean getCubeMode(){
        if(!cubeMode && operator.getLeftStickButton()){
            cubeMode = true;
        } else if(cubeMode && operator.getRightStickButton()){
            cubeMode = false;
        }

        return cubeMode;
    }

    public double armShoulder(){     
        if(Math.abs(operator.getLeftY()) > 0.1){
            return operator.getLeftY() * 0.5;
        } else {
            return 0;
        }
    }

    public double armExtension(){
        if(Math.abs(operator.getRightY()) > 0.1){
            return operator.getRightY() * 0.5;
        } else {
            return 0;
        }
    }

    @Override
    public boolean getPickUpObject() {
        // TODO Auto-generated method stub
        return driver.getAButton();
    }

    @Override
    public boolean hopperOverrideLeft() {
        return operator.getXButton();
    }

    @Override
    public boolean hopperOverrideRight() {
        return operator.getBButton();
    }
}