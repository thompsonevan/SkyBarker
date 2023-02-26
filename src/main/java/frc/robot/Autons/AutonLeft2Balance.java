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

// public class AutonLeft2Balance extends AutonBase{
//     enum AutoState {
//         firstPlace,
//         driveToObject1,
//         driveToObject2,
//         pickUpObject,
//         driveBack,
//         secondPlace,
//         chargingStation,
//         end
//     }

//     public AutoState autoState;

//     public Timer timer = new Timer();

//     int point = 0;

//     Trajectory trajectory;

//     List<Pose2d> path = List.of(new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
//                                 new Pose2d(new Translation2d(Units.feetToMeters(14),-Units.feetToMeters(5)), Rotation2d.fromDegrees(0)),
//                                 new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
//                                 new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)));

//     public AutonLeft2Balance(){
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
//                     trajectory = createTrajectory(path.get(point), path.get(point+1));
            
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

//                 if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .1 &&
//                    Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .1 &&
//                    Math.abs(Drivetrain.getPose().getRotation().getDegrees() - path.get(point).getRotation().getDegrees()) < 5){
//                     timer.reset();

//                     autoState = AutoState.pickUpObject;
//                 }
//             break;
//             case driveToObject2:
                
//             break;
//             case pickUpObject:
//                 driving = false;
//                 armPos = ArmPos.packagePos;
//                 intakeOn = true;

//                 if(timer.get() > 1){
//                     trajectory = createTrajectory(path.get(point), path.get(point+1));
            
//                     Drivetrain.setPose(path.get(point), path.get(point).getRotation());

//                     point++;

//                     timer.reset();

//                     autoState = AutoState.driveBack;
//                 }
//             break;
//             case driveBack:
//                 driving = true;
//                 armPos = ArmPos.packagePos;
//                 intakeOn = false;

//                 desState = getState(timer.get(), trajectory, path.get(point).getRotation());

//                 if(Math.abs(Drivetrain.getPose().getX() - path.get(point).getX()) < .1 &&
//                 Math.abs(Drivetrain.getPose().getY() - path.get(point).getY()) < .1 &&
//                 Math.abs(Drivetrain.getPose().getRotation().getDegrees() - path.get(point).getRotation().getDegrees()) < 5){
//                     timer.reset();

//                     autoState = AutoState.secondPlace;
//                 }
//             break;
//             case secondPlace:
//                 driving = false;
//                 armPos = ArmPos.middleNode;
//                 intakeOn = false;

//                 if(timer.get() > 3){
//                     trajectory = createTrajectory(path.get(point), path.get(point+1));
            
//                     Drivetrain.setPose(path.get(point), path.get(point).getRotation());

//                     point++;

//                     timer.reset();

//                     autoState = AutoState.chargingStation;
//                 }
//             break;
//             case chargingStation:
//                 driving = true;
//                 armPos = ArmPos.packagePos;
//                 intakeOn = false;

//                 desState = getState(timer.get(), trajectory, path.get(point).getRotation());
                        
//             break;
//             case end:
                                    
//             break;
//         }
//     }
// }
