// package frc.robot.Autons;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.sensors.Pigeon;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Arm.ArmPos;

// import java.util.List;

// import javax.sound.midi.Track;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.PathPoint;
// import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.Trajectory.State;

// public class AutonLeft extends AutonBase{
//     enum AutoState {
//         firstPlace,
//         driveToObject,
//         pickUpObject,
//         driveBack,
//         secondPlace,
//         chargingStation,
//         end
//     } 

//     public AutoState autoState;

//     public Timer timer = new Timer();

//     Trajectory trajectory;
//     Trajectory trajectory1;

//     public AutonLeft(){
//         autoState = AutoState.firstPlace;

//         timer.reset();
//         timer.start();

//         // trajectory = PathPlanner.loadPath("path1", new PathConstraints(3,2));
//         // trajectory1 = PathPlanner.loadPath("path2", new PathConstraints(3,2));

//         // trajectory = PathPlanner.generatePath(
//         //     new PathConstraints(4, 2),
//         //     new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90)), // position, heading(direction of travel), holonomic rotation
//         //     new PathPoint(new Translation2d(3, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
//         // );

//         trajectory = TrajectoryGenerator.generateTrajectory(
//             new Pose2d(0, 0, new Rotation2d(90)), 
//             List.of(), 
//             new Pose2d(3, 0, new Rotation2d(0)), 
//             new TrajectoryConfig(4, 2));

//         desState = new State();

//         initalPose = trajectory.getInitialPose();
//     }

//     public void reset(){
//         autoState = AutoState.firstPlace;

//         desState = new State();

//         initalPose = trajectory.getInitialPose();

//         Drivetrain.setPose(new Pose2d(0,0,Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(90));

//         timer.reset();
//         timer.start();
//     }

//     State state;

//     Pose2d newPose;

//     public void runAuto(){
//         SmartDashboard.putString("Auton State", autoState.toString());

//         switch(autoState){
//             case firstPlace:
//                 armPos = ArmPos.topNode;
//                 driving = false;

//                 if(timer.get() > 5){
//                     armPos = ArmPos.packagePos;
//                     autoState = AutoState.driveToObject;
//                     Drivetrain.stopMotors();
//                     timer.reset();
//                 }
//             break;
//             case driveToObject:
//                 driving = true;
//                 state = trajectory.sample(timer.get());

//                 newPose = new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), new Rotation2d(0));
//                 desState = new State(timer.get(), state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq, newPose, state.curvatureRadPerMeter);

//                 // if((Drivetrain.getPose().getX() - trajectory.getEndState().poseMeters.getX()) < .1
//                 //     && (Drivetrain.getPose().getY() - trajectory.getEndState().poseMeters.getY()) < .1){
//                 //         autoState = AutoState.pickUpObject;
//                 //         Drivetrain.stopMotors();
//                 //         timer.reset();
//                 // }
//                 if(timer.get() > trajectory.getTotalTimeSeconds()){
//                     autoState = AutoState.pickUpObject;
//                     Drivetrain.stopMotors();
//                     timer.reset();
//                 }
//             break;
//             case pickUpObject:
//                 driving = false;
//                 intakeOn = true;
//                 pickUpObject = true;

//                 Drivetrain.setPose(Drivetrain.getPose(), Pigeon.getRotation2d());

//                 if(timer.get() > 5){
//                     intakeOn = false;
//                     pickUpObject = false;
//                     autoState = AutoState.driveBack;
//                     Drivetrain.stopMotors();
//                     timer.reset();
//                     trajectory1 = TrajectoryGenerator.generateTrajectory(
//                         new Pose2d(3, 0, new Rotation2d()), 
//                         List.of(), 
//                         new Pose2d(0, 0, new Rotation2d()), 
//                         new TrajectoryConfig(4, 2));
//                     Drivetrain.setPose(new Pose2d(3,0, Pigeon.getRotation2d()), Pigeon.getRotation2d());
//                 }
//             break;
//             case driveBack:
//                 driving = true;
//                 state = trajectory.sample(timer.get());

//                 newPose = new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(), new Rotation2d(0));
//                 desState = new State(timer.get(), state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq, newPose, state.curvatureRadPerMeter);

//                 if(timer.get() > trajectory1.getTotalTimeSeconds()){
//                     autoState = AutoState.secondPlace;
//                     Drivetrain.stopMotors();
//                     timer.reset();
//                 }
//             break;
//             case secondPlace:
//                 driving = false;
//                 armPos = ArmPos.middleNode;

//                 if(timer.get() > 5){
//                     autoState = AutoState.end;
//                     Drivetrain.stopMotors();
//                     timer.reset();
//                 }
//             break;
//             case chargingStation:

//             break;
//             case end:
//                 armPos = ArmPos.packagePos;
//                 Drivetrain.stopMotors();
//             break;
//         }

//         SmartDashboard.putNumber("Des X", desState.poseMeters.getX());
//         SmartDashboard.putNumber("Des Y", desState.poseMeters.getY());
//         SmartDashboard.putNumber("Des Theta", desState.poseMeters.getRotation().getDegrees());
//     }
// }
