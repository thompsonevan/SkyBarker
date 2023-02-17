package frc.robot;

import org.hotutilites.hotlogger.HotLogger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.Auton1;
import frc.robot.Autons.Auton67;
import frc.robot.Autons.AutonLeft;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {
    private TeleopCommander teleopCommander;
    private Drivetrain drivetrain;
    private Hopper hopper;
    private Arm arm;
    // private Intake intake;
    private Pigeon pigeon;
    private Camera camera;
    private AutonCommader autonCommader;
    private Auton1 auton;
    private Auton67 auton67;
    private AutonLeft autonLeft;

    private int autonSelection = 2;

    @Override
    public void robotInit() {
        HotLogger.Setup("Theta", "Left Front Absolute", "Left Front Assumed",
        "Right Front Absolute", "Right Front Assumed",
        "Left Rear Absolute", "Left Rear Assumed",
        "Right Rear Absolute", "Right Rear Assumed",
        "TargetX", "TargetY", "TargetTheta", "Robot State Theta", "poseX", "poseY",
        "Shoulder Absolute Pos", "Shoulder Motor Pos", "Extension Pos", "Elbow Absolute Pos", "Elbow Motor Pos",
        "Shoulder Desired Pos", "Extension Desired Pos", "Elbow Desired Pos");

        teleopCommander = new TeleopCommander();
        pigeon = new Pigeon();
        camera = new Camera();
        drivetrain = new Drivetrain();
        autonCommader = new AutonCommader();
        arm = new Arm();
        auton = new Auton1();
        auton67 = new Auton67();
        autonLeft = new AutonLeft();
        // intake = new Intake();
        hopper = new Hopper();
    }

    @Override
    public void robotPeriodic() {
        arm.logData();
        camera.logData();
        pigeon.logData();
        // intake.logData();
    }

    @Override
    public void disabledInit() {
        drivetrain.zero();
        arm.armZeroSensorPos();
        // drivetrain.setBrakeMode(false);
    }

    @Override
    public void disabledPeriodic() {
        drivetrain.disabled();
        arm.armPercentOutZero();
        arm.coastMode();
    }

    @Override
    public void autonomousInit() {
        // drivetrain.setBrakeMode(true);

        if(autonSelection == 0){
            autonCommader.initAuton(auton);
        } else if(autonSelection == 1){
            autonCommader.initAuton(auton67);
        } else if(autonSelection == 2){
            autonCommader.initAuton(autonLeft);
        } else {
            autonCommader.initAuton(auton);
        }

        autonCommader.auton.reset();
        drivetrain.zero();
        Drivetrain.setPose(autonCommader.getInitalState().poseMeters, autonCommader.getInitalState().holonomicRotation);
    }

    @Override
    public void autonomousPeriodic() {
        pigeon.enabledAction(teleopCommander);
        autonCommader.runAuto();
        drivetrain.autonAction(autonCommader);
        arm.action(autonCommader);
    }

    private double[] rip2;
    
    @Override
    public void teleopInit() {
        // drivetrain.setBrakeMode(true);

        drivetrain.zero();
        Pigeon.zeroSensor();
        arm.armZeroSensorPos();
    }

    @Override
    public void teleopPeriodic() {
        pigeon.enabledAction(teleopCommander);
        drivetrain.teleAction(   teleopCommander);
        rip2 = teleopCommander.getIntakePosition();
        // intake.IntakePeriodic(teleopCommander);
        SmartDashboard.putNumber("rip1", rip2[0]);
        SmartDashboard.putNumber("rip2", rip2[1]);
        arm.action(teleopCommander);
        arm.brakeMode();
        hopper.enabled(teleopCommander);
    }
}
