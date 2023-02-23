package frc.robot.Autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmPos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class AutoBalance extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject,
        pickUpObject,
        driveBack,
        secondPlace,
        chargingStation,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    Trajectory trajectory;

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(1.75,4.45), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(6.62,4.63), Rotation2d.fromDegrees(0)),
                                new Pose2d(new Translation2d(1.75,4.45), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(3.9,2.9), Rotation2d.fromDegrees(-90)));

    public AutoBalance(){
        reset();
    }

    public void reset(){
        autoState = AutoState.firstPlace;

        point = 0;

        desState = new State();

        timer.reset();
        timer.start();
    }

    public void runAuto(){
        driving = false;
        autoBalance = true;
    }
}
