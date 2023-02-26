// package frc.robot.Autons;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.sensors.Pigeon;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Arm.ArmPos;

// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.Trajectory.State;
// import edu.wpi.first.math.util.Units;

// public class TestAuto extends AutonBase{
//     enum AutoState {
//         firstPlace,
//         driveToObject1,
//         pause,
//         driveToObject2,
//         score2,
        
//         end
//     }

//     public AutoState autoState;

//     public Timer timer = new Timer();

//     int point = 0;

//     Trajectory trajectory;

//     List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
//                                 new Pose2d(new Translation2d(Units.feetToMeters(14),-Units.feetToMeters(5)), Rotation2d.fromDegrees(-45)),
//                                 new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)));
    
//     public TestAuto(){
//         reset();
//     }

//     public void reset(){
//         autoState = AutoState.firstPlace;

//         point = 0;

//         desState = new State();

//         timer.reset();
//         timer.start();
//     }

//     public void runAuto(){
//         switch(autoState){
//             case firstPlace:
//                 driving = false;
//                 armPos = ArmPos.topNode;
//                 intakeOn = false;

//                 if(timer.get() > 3){
//                     trajectory = createTrajectory(path.get(point), path.get(point+1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-45));
            
//                     Drivetrain.setPose(path.get(point), path.get(point).getRotation());

//                     point++;

//                     timer.reset();

//                     autoState = AutoState.driveToObject1;
//                 }
//             break;
//             case driveToObject1:
//                 driving = true;
//                 armPos = ArmPos.packagePos;
//                 intakeOn = true;
                
//                 desState = getState(timer.get(), trajectory, path.get(point).getRotation());

//                 if(timer.get() > trajectory.getTotalTimeSeconds()){
//                     timer.reset();

//                     autoState = AutoState.pause;
//                 }
//             break;
//             case pause:
//                 driving = false;
//                 if(timer.get() > 1){
//                     Pose2d newEnd = new Pose2d(path.get(point+1).getTranslation(), Drivetrain.getPose().getRotation());
//                     trajectory = createTrajectory(path.get(point), newEnd);

//                     point++;

//                     timer.reset();

//                     autoState = AutoState.driveToObject2;
//                 }
//             break;
//             case driveToObject2:
//                 driving = true;
//                 armPos = ArmPos.packagePos;
//                 intakeOn = true;
                
//                 desState = getState(timer.get(), trajectory, path.get(point).getRotation());

//                 if(timer.get() > trajectory.getTotalTimeSeconds()){                    
//                     timer.reset();

//                     autoState = AutoState.end;
//                 }
//             break;
//             case end:
//                 driving = false;
//             break;
//         }
//     }
// }
