// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Constants;
import frc.robot.sensors.Camera2;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Camera2 limelight;
  Pigeon pigeon;
  Drivetrain drivetrain;
  private XboxController driver;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    driver = new XboxController(0);
    limelight = new Camera2("limelight");
    if (Constants.realBot) {
      pigeon = new Pigeon(Constants.DRIVETRAIN_PIGEON_ID,"None");
    } else {
      pigeon = new Pigeon(Constants.DRIVETRAIN_PIGEON_ID);
    }
    pigeon.setYaw(-180);
    drivetrain = new Drivetrain(pigeon, -180);
  }

  @Override
  public void robotPeriodic() {
    pigeon.updatePose();
    drivetrain.updatePose(limelight);
    SmartDashboard.putNumber("FPGA Time", Timer.getFPGATimestamp());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */

   Trajectory trja1;
   Timer timer = new Timer();
  private Pose2d endingPose;
  @Override
  public void autonomousInit() {

    drivetrain.resetSteer();

    SmartDashboard.putString("Robot Mode", "Autonomous");
    if (limelight.aprilTagsDetected()) {
      Pose2d startingPosition = limelight.getBotPose();
      endingPose = new Pose2d(1.85, 1.6,  Rotation2d.fromDegrees(180));

      double headingAngle = Math.toDegrees(Math.atan2(endingPose.getY()-startingPosition.getY(),endingPose.getX()-startingPosition.getX()));

      SmartDashboard.putNumber("Heading Angle", headingAngle);

      
      SmartDashboard.putNumber("Initial Theta", startingPosition.getRotation().getDegrees());
      SmartDashboard.putNumber("Initial X", startingPosition.getX());
      SmartDashboard.putNumber("Initial Y", startingPosition.getY());

      
      SmartDashboard.putNumber("End Theta", endingPose.getRotation().getDegrees());
      SmartDashboard.putNumber("End X", endingPose.getX());
      SmartDashboard.putNumber("End Y", endingPose.getY());

      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(7,2.5)
      .setKinematics(Constants.KINEMATICS);
      
      trja1 = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(startingPosition.getTranslation(), Rotation2d.fromDegrees(headingAngle)),
      new Pose2d(endingPose.getTranslation(), Rotation2d.fromDegrees(headingAngle))), trajectoryConfig);
    
    }
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    drivetrain.driveToPoint(trja1.sample(timer.get()),endingPose.getRotation());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    drivetrain.resetSteer();
    SmartDashboard.putString("Robot Mode", "TeleOp");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drivetrain.driveWithController(driver);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    SmartDashboard.putString("Robot Mode", "Disabled");
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
