package frc.robot;

import org.hotutilites.hotlogger.HotLogger;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.Auton1;
import frc.robot.Autons.Auton67;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.swing.text.StyleContext.SmallAttributeSet;

public class Robot extends TimedRobot {
    private TeleopCommander teleopCommander;
    private Drivetrain drivetrain;
    private Pigeon pigeon;
    private Camera camera;
    private AutonCommader autonCommader;
    private Auton1 auton;
    private Arm arm;
    private Auton67 auton67;

    private int autonSelection = 0;


    @Override
    public void robotInit() {
        HotLogger.Setup("Theta", "Left Front Absolute", "Left Front Assumed",
        "Right Front Absolute", "Right Front Assumed",
        "Left Rear Absolute", "Left Rear Assumed",
        "Right Rear Absolute", "Right Rear Assumed",
        "TargetX", "TargetY", "TargetTheta", "Robot State Theta", "poseX", "poseY");

        teleopCommander = new TeleopCommander();
        pigeon = new Pigeon();
        camera = new Camera();
        drivetrain = new Drivetrain();
        autonCommader = new AutonCommader();
        arm = new Arm();
        auton = new Auton1();
        auton67 = new Auton67();
    }

    @Override
    public void robotPeriodic() {
        arm.logData();
        camera.logData();
        pigeon.logData();
    }

    @Override
    public void disabledInit() {
        drivetrain.zero();
        arm.armZeroSensorPos();
    }

    @Override
    public void disabledPeriodic() {
        drivetrain.disabled();
        arm.armPercentOutZero();
        arm.coastMode();
    }

    @Override
    public void autonomousInit() {
        // if(autonSelection == 0){
        //     autonCommader.initAuton(auton);
        // } else if(autonSelection == 1){
        //     autonCommader.initAuton(auton67test);
        // } else {
        //     autonCommader.initAuton(auton);
        // }

        autonCommader.initAuton(auton67);
        autonCommader.auton.reset();
        drivetrain.zero();
        Drivetrain.setPose(autonCommader.getInitalState().poseMeters, autonCommader.getInitalState().holonomicRotation);
    }

    @Override
    public void autonomousPeriodic() {
        pigeon.enabledAction(teleopCommander);
        autonCommader.runAuto();
        drivetrain.autonAction(autonCommader);
    }

    @Override
    public void teleopInit() {
        drivetrain.zero();
        Pigeon.zeroSensor();
        arm.armZeroSensorPos();
    }

    @Override
    public void teleopPeriodic() {
        pigeon.enabledAction(teleopCommander);
        drivetrain.teleAction(teleopCommander);
        arm.armTeleAction(teleopCommander);
        arm.brakeMode();
    }
}
