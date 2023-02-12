package frc.robot;

import org.hotutilites.hotlogger.HotLogger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.Auton1;
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
    private AutonCommader autonCommader;
    private Auton1 auton;
    private Arm arm;

    @Override
    public void robotInit() {
        HotLogger.Setup("Theta", "Left Front Absolute", "Left Front Assumed",
        "Right Front Absolute", "Right Front Assumed",
        "Left Rear Absolute", "Left Rear Assumed",
        "Right Rear Absolute", "Right Rear Assumed",
        "TargetX", "TargetY", "TargetTheta", "Robot State Theta", "poseX", "poseY");

        teleopCommander = new TeleopCommander();
        pigeon = new Pigeon();
        drivetrain = new Drivetrain();
        autonCommader = new AutonCommader();
        arm = new Arm();
    }

    @Override
    public void robotPeriodic() {
        arm.logData();
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
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("simple", new PathConstraints(1,1));

        Pigeon.zeroSensor(trajectory.getInitialPose().getRotation().getDegrees());
        drivetrain.setPose(trajectory.getInitialPose());
        auton = new Auton1();
        autonCommader.initAuton(auton);
        
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
        SmartDashboard.putBoolean("ButtonA", teleopCommander.getArmPosition1());
    }
}
