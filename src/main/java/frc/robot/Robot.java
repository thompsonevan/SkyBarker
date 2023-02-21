package frc.robot;

import java.util.List;

import org.hotutilites.hotlogger.HotLogger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.AutonLeft1Balance;
import frc.robot.Autons.AutonLeft2Balance;
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
    private Intake intake;
    private Pigeon pigeon;
    private Camera camera;
    private AutonCommader autonCommader;
    private AutonLeft2Balance autonLeft;
    private AutonLeft1Balance autonLeft1Balance;

    private int autonSelection = 1;

    @Override
    public void robotInit() {
        HotLogger.Setup("Theta", "Left Front Absolute", "Left Front Assumed",
        "Right Front Absolute", "Right Front Assumed",
        "Left Rear Absolute", "Left Rear Assumed",
        "Right Rear Absolute", "Right Rear Assumed",
        "TargetX", "TargetY", "TargetTheta", "Robot State Theta", "poseX", "poseY",
        "Shoulder Absolute Pos", "Shoulder Motor Pos", "Extension Pos", "Elbow Absolute Pos", "Elbow Motor Pos",
        "Shoulder Desired Pos", "Extension Desired Pos", "Elbow Desired Pos",
        "HopSensor Bottom", "HopSensor Left", "HopSensor Right", "HopSensor Top", "Hopper Override");


        intake = new Intake();
        teleopCommander = new TeleopCommander();
        pigeon = new Pigeon();
        camera = new Camera();
        drivetrain = new Drivetrain();
        autonCommader = new AutonCommader();
        arm = new Arm();
        autonLeft1Balance = new AutonLeft1Balance();
        autonLeft = new AutonLeft2Balance();
        hopper = new Hopper();
    }

    @Override
    public void robotPeriodic() {
        arm.logData();
        camera.logData();
        pigeon.logData();
        intake.logData();   
        hopper.logData();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("FPGA Time", Timer.getFPGATimestamp());
        drivetrain.updatePose();
    }

    @Override
    public void disabledInit() {
        drivetrain.zero();
        arm.armZeroSensorPos();
        SmartDashboard.putString("Robot Mode", "Disabled");
    }

    @Override
    public void disabledPeriodic() {
        arm.armPercentOutZero();
        arm.coastMode();
    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Robot Mode", "Autonomous");

        if(autonSelection == 0){
            // autonCommader.initAuton(auton);
        } else if(autonSelection == 1){
            autonCommader.initAuton(autonLeft1Balance);
        } else if(autonSelection == 2){
            autonCommader.initAuton(autonLeft);
        } else {
            // autonCommader.initAuton(auton);
        }

        drivetrain.zero();
        autonCommader.auton.reset();
        // Drivetrain.setPose(new Pose2d(0,0, Rotation2d.fromDegrees(-180)), Rotation2d.fromDegrees(-180));
    }

    @Override
    public void autonomousPeriodic() {
        autonCommader.runAuto();
        pigeon.enabledAction(teleopCommander);
        drivetrain.autonAction(autonCommader);
        arm.action(autonCommader);
        intake.IntakePeriodic(autonCommader);
        hopper.HopperPeriodic(autonCommader);
    }
    
    @Override
    public void teleopInit() {
        SmartDashboard.putString("Robot Mode", "Teleop");

        drivetrain.zero();
        Pigeon.zeroSensor();
        arm.armZeroSensorPos();
    }

    @Override
    public void teleopPeriodic() {
        pigeon.enabledAction(teleopCommander);
        drivetrain.teleAction(   teleopCommander);
        intake.IntakePeriodic(teleopCommander);
        arm.action(teleopCommander);
        arm.brakeMode();
        hopper.HopperPeriodic(teleopCommander);
    }
}
