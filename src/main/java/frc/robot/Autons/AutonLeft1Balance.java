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

public class AutonLeft1Balance extends AutonBase{
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

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(0,-2), Rotation2d.fromDegrees(-90)), // x - 2.15
                                new Pose2d(new Translation2d(2,-2), Rotation2d.fromDegrees(-90))); // y - -1.55

    public AutonLeft1Balance(){
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
        switch(autoState){
            case firstPlace:
                driving = false;
                armPos = ArmPos.topNode;
                intakeOn = false;

                if(timer.get() > 3){
                    trajectory = createTrajectory(path.get(point), path.get(point+1));
            
                    Drivetrain.setPose(path.get(point), path.get(point).getRotation());

                    point++;

                    timer.reset();

                    autoState = AutoState.chargingStation1;
                }
            break;
            case chargingStation1:
                driving = true;
                armPos = ArmPos.packagePos;
                intakeOn = false;

                desState = getState(timer.get(), trajectory, path.get(point).getRotation());

                if(timer.get() > trajectory.getTotalTimeSeconds()){
                    trajectory = createTrajectory(path.get(point), path.get(point+1));
            
                    Drivetrain.setPose(path.get(point), path.get(point).getRotation());

                    point++;

                    timer.reset();

                    autoState = AutoState.chargingStation2;
                }
            break;
            case chargingStation2:
                driving = true;
                armPos = ArmPos.packagePos;
                intakeOn = false;

                desState = getState(timer.get(), trajectory, path.get(point).getRotation());
            case end:

            break;
        }
    }
}
