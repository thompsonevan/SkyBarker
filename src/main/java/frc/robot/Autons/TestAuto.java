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
import edu.wpi.first.math.util.Units;

public class TestAuto extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject1,
        pause,
        driveToObject2,
        score2,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    Trajectory trajectory;

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
                                new Pose2d(new Translation2d(4.82,.45), Rotation2d.fromDegrees(0)), //4.82, .5
                                new Pose2d(new Translation2d(0,.5), Rotation2d.fromDegrees(-90)));
    
    public TestAuto(){
        reset();
    }

    public void reset(){
        autoState = AutoState.firstPlace;

        point = 0;

        desState = new State();
        targetTheta = Rotation2d.fromDegrees(-90);

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
                    trajectory = createTrajectory(path.get(point), path.get(point+1),
                    Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-15));
            
                    point++;

                    timer.reset();

                    autoState = AutoState.driveToObject1;
                }
            break;
            case driveToObject1:
                driving = true;
                armPos = ArmPos.packagePos;
                intakeOn = false;
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(timer.get() > trajectory.getTotalTimeSeconds()){
                    timer.reset();

                    autoState = AutoState.pause;
                }
            break;
            case pause:
                driving = false;
                if(timer.get() > 1){
                    trajectory = createTrajectory(path.get(point), path.get(point+1), 
                    Rotation2d.fromDegrees(-10 + 180), Rotation2d.fromDegrees(10 + 180));

                    point++;

                    timer.reset();

                    autoState = AutoState.driveToObject2;
                }
            break;
            case driveToObject2:
                driving = true;
                armPos = ArmPos.packagePos;
                intakeOn = false;
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(timer.get() > trajectory.getTotalTimeSeconds()){                    
                    timer.reset();

                    autoState = AutoState.end;
                }
            break;
            case end:
                driving = false;
            break;
        }
        SmartDashboard.putString("Auto State", autoState.toString());
    }
}
