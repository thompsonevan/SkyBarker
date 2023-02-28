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

// Big edit


public class RedAutoLeft extends AutonBase{
    enum AutoState {
        firstPlace,
        driveToObject1,
        pause,
        driveToObject2,
        score2,
        driveToBalance,
        balance,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    int point = 0;

    List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(90)),
                                new Pose2d(new Translation2d(5.15,.6), Rotation2d.fromDegrees(0)), //4.82, .5
                                new Pose2d(new Translation2d(0,.5), Rotation2d.fromDegrees(90)),
                                new Pose2d(new Translation2d(.68,-1.26), Rotation2d.fromDegrees(90)));

    Trajectory trajectory;

    public RedAutoLeft(){
        reset();
    }

    public void reset(){
        desState = new State();
        targetTheta = Rotation2d.fromDegrees(90);

        point = 0;
        
        timer.reset();
        timer.start();

        autoState = AutoState.firstPlace;
    }

    public void runAuto(){
        switch(autoState){
            case firstPlace:
                driving = false;
                if(timer.get() < 2.5){
                    if(timer.get() < 2.25){
                        gripperSpeed = -.4;
                    }else{
                        gripperSpeed = .4;
                    }
                    armPos = ArmPos.topNode;
                } else {
                    gripperSpeed = 0;
                    armPos = ArmPos.intake;

                    trajectory = createTrajectory(path.get(point), path.get(point+1),
                    Rotation2d.fromDegrees(40), Rotation2d.fromDegrees(-10));
            
                    point++;

                    timer.reset();

                    autoState = AutoState.driveToObject1;
                }
            break;
            case driveToObject1:
                driving = true;
                armPos = ArmPos.intake;
                if(timer.get() > 1.5){
                    intakeOn = true;
                }
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(timer.get() > trajectory.getTotalTimeSeconds()){
                    timer.reset();

                    autoState = AutoState.pause;
                }
            break;
            case pause:
                driving = false;
                intakeOn = false;
                armPos = ArmPos.intake;
                if(timer.get() > .5){
                    trajectory = createTrajectory(path.get(point), path.get(point+1), 
                    Rotation2d.fromDegrees(-10+180), Rotation2d.fromDegrees(10+180));

                    overrideNegSide = true;

                    point++;

                    timer.reset();

                    autoState = AutoState.driveToObject2;
                }
            break;
            case driveToObject2:
                driving = true;
                intakeOn = false;

                if(timer.get() < .5){
                    armPos = ArmPos.packagePos;
                } else {
                    armPos = ArmPos.topNode;
                }

                gripperSpeed = -.4;
                
                desState = trajectory.sample(timer.get());
                targetTheta = path.get(point).getRotation();

                if(timer.get() > trajectory.getTotalTimeSeconds()){                    
                    timer.reset();
                    
                    gripperSpeed = 0;

                    autoState = AutoState.score2;
                }
            break;
            case score2:
                driving = false;
                if(timer.get() < 1){
                    gripperSpeed = .2;
                } else {
                    armPos = ArmPos.packagePos;
                }
            case end:
                driving = false;
                autoBalance = false;
            break;
        }
        SmartDashboard.putString("Auton State", autoState.toString());
    }
}