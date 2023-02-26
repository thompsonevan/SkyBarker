package frc.robot.Autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm.ArmPos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class DriveToPoint extends AutonBase{
    enum AutoState {
        firstPlace,
        chargingStation1,
        chargingStation2,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    Trajectory trajectory;

    public DriveToPoint(){
        reset();
    }

    Pose2d start = new Pose2d(0,0,Rotation2d.fromDegrees(-90));
    Pose2d end = new Pose2d(2,2, Rotation2d.fromDegrees(-90));

    public void reset(){
        // Drivetrain.setPose(start);
        

        // trajectory = createTrajectory(start, end);

        desState = new State();

        timer.reset();
        timer.start();
    }

    public void runAuto(){
        // desState = getState(timer.get(), trajectory, end.getRotation());
        desState = trajectory.sample(timer.get());
        targetTheta = Rotation2d.fromDegrees(-90);
    }
}
