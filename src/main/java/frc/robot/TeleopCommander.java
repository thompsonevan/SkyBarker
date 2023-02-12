package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.*;

public class TeleopCommander extends RobotCommander{

    private static XboxController driver;
    private static XboxController operator;

    public TeleopCommander() {
        driver = new XboxController(0);
        operator = new XboxController(1);
    }

    @Override
    public double getForwardCommand() {
        return -(modifyAxis(driver.getLeftY()) * MAX_VELOCITY_METERS_PER_SECOND) * .2;
    }

    @Override
    public double getStrafeCommand() {
        return -(modifyAxis(driver.getLeftX()) * MAX_VELOCITY_METERS_PER_SECOND) * .2;
    }

    @Override
    public double getTurnCommand() {
        double value = deadband(Math.abs(driver.getRightX()) * driver.getRightX(), 0.13, 0.4) * (MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

        return -value;
    }

    @Override
    public boolean getResetIMU() {
        return driver.getBackButton();
    }
 
    
    private static double deadband(double value, double deadband, double maxRange){
        if(Math.abs(value) < deadband){
            return 0;
        } else if (value < 0) {
            return  ((value + deadband)/(1.0 - deadband)) * maxRange;
        } else {
            return  ((value - deadband)/(1.0 - deadband)) * maxRange;
        }
    }
  
    private static double modifyAxis(double value) {
        boolean deadband = 0.13 > Math.sqrt(Math.pow(driver.getLeftX(), 2) + Math.pow(driver.getLeftY(), 2));

        if (deadband) {
            return 0;
        } else {
            return Math.abs(value) * value;
        }
    }

    @Override
    public boolean getArmPosition1(){
        return operator.getAButton();
    }

    @Override
    public boolean getArmPosition2(){
        return operator.getBButton();
    }

    public boolean getArmPosition3(){
        return operator.getYButton();
    }

    public boolean getArmPositionPackage(){
        return operator.getBackButton();
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
}