package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutonCommader;
import frc.robot.TeleopCommander;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Pigeon;

import static frc.robot.Constants.*;

public class Drivetrain{
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private static TalonFX frontLeftDrive;
    private static TalonFX frontRightDrive;
    private static TalonFX backLeftDrive;
    private static TalonFX backRightDrive;

    // private final TalonFX frontLeftSteer;
    // private final TalonFX frontRightSteer;
    // private final TalonFX backLeftSteer;
    // private final TalonFX backRightSteer;

    // private final AbsoluteEncoder frontLeftEncoder;
    // private final AbsoluteEncoder frontRightEncoder;
    // private final AbsoluteEncoder backLeftEncoder;
    // private final AbsoluteEncoder backRightEncoder;

    static SwerveModulePosition[] positions;

    public static SwerveDrivePoseEstimator poseEstimator;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private SwerveModuleState[] states;

    Timer timer = new Timer();

    // private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    //                 new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    //                 new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
    //                 new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    //                 new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    // private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    //     new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    //     new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
    //     new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    //     new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));


    private HolonomicDriveController holonomicController;

    double frontLeftPos;
    double frontRightPos;
    double backLeftPos;
    double backRightPos;

    Pose2d scoringPose = new Pose2d(-6.12 , -3.57, Rotation2d.fromDegrees(180));

    // private SwerveDrivePoseEstimator poseExstimator;

    public Drivetrain(){
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        if(realBot){
            frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                    tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                    .withSize(2, 4)
                                    .withPosition(0, 0),
                                    Mk4iSwerveModuleHelper.GearRatio.L2,
                    FRONT_LEFT_MODULE_DRIVE_MOTOR,
                    FRONT_LEFT_MODULE_STEER_MOTOR,
                    FRONT_LEFT_MODULE_STEER_ENCODER,
                    FRONT_LEFT_MODULE_STEER_OFFSET);

            frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(2, 0),
                                            Mk4iSwerveModuleHelper.GearRatio.L2,
                            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                            FRONT_RIGHT_MODULE_STEER_MOTOR,
                            FRONT_RIGHT_MODULE_STEER_ENCODER,
                            FRONT_RIGHT_MODULE_STEER_OFFSET);

            backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(4, 0),
                            Mk4iSwerveModuleHelper.GearRatio.L2,
                            BACK_LEFT_MODULE_DRIVE_MOTOR,
                            BACK_LEFT_MODULE_STEER_MOTOR,
                            BACK_LEFT_MODULE_STEER_ENCODER,
                            BACK_LEFT_MODULE_STEER_OFFSET);

            backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(6, 0),
                            Mk4iSwerveModuleHelper.GearRatio.L2,
                            BACK_RIGHT_MODULE_DRIVE_MOTOR,
                            BACK_RIGHT_MODULE_STEER_MOTOR,
                            BACK_RIGHT_MODULE_STEER_ENCODER,
                            BACK_RIGHT_MODULE_STEER_OFFSET);

            frontLeftDrive = frontLeftModule.getDriveMotor();
            frontRightDrive = frontRightModule.getDriveMotor();
            backLeftDrive = backLeftModule.getDriveMotor();
            backRightDrive = backRightModule.getDriveMotor();

            frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        } else {
            frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(0, 0),
                                Mk4SwerveModuleHelper.GearRatio.L2,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET);

            frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(2, 0),
                                            Mk4SwerveModuleHelper.GearRatio.L2,
                            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                            FRONT_RIGHT_MODULE_STEER_MOTOR,
                            FRONT_RIGHT_MODULE_STEER_ENCODER,
                            FRONT_RIGHT_MODULE_STEER_OFFSET);

            backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(4, 0),
                                            Mk4SwerveModuleHelper.GearRatio.L2,
                            BACK_LEFT_MODULE_DRIVE_MOTOR,
                            BACK_LEFT_MODULE_STEER_MOTOR,
                            BACK_LEFT_MODULE_STEER_ENCODER,
                            BACK_LEFT_MODULE_STEER_OFFSET);

            backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                            .withSize(2, 4)
                                            .withPosition(6, 0),
                                            Mk4SwerveModuleHelper.GearRatio.L2,
                            BACK_RIGHT_MODULE_DRIVE_MOTOR,
                            BACK_RIGHT_MODULE_STEER_MOTOR,
                            BACK_RIGHT_MODULE_STEER_ENCODER,
                            BACK_RIGHT_MODULE_STEER_OFFSET);

            frontLeftDrive = frontLeftModule.getDriveMotor();
            frontRightDrive = frontRightModule.getDriveMotor();
            backLeftDrive = backLeftModule.getDriveMotor();
            backRightDrive = backRightModule.getDriveMotor();

            frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        }



        // frontLeftSteer = frontLeftModule.getSteerMotor();
        // frontRightSteer = frontRightModule.getSteerMotor();
        // backLeftSteer = backLeftModule.getSteerMotor();
        // backRightSteer = backRightModule.getSteerMotor();

        // frontLeftEncoder = frontLeftModule.getEncoder();
        // frontRightEncoder = frontRightModule.getEncoder();
        // backLeftEncoder = backLeftModule.getEncoder();
        // backRightEncoder = backRightModule.getEncoder();

        positions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

        positions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());
        positions[0].distanceMeters = frontLeftPos;

        positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
        positions[1].distanceMeters = frontRightPos;

        positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[2].distanceMeters = backLeftPos;

        positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
        positions[3].distanceMeters = backRightPos;

        states = kinematics.toSwerveModuleStates(new ChassisSpeeds());

        ProfiledPIDController thetaController = new ProfiledPIDController(.2, .3, 0,
                                                new TrapezoidProfile.Constraints(6.28, 3.14));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        holonomicController = new HolonomicDriveController(
            new PIDController(1.5, .5, 0),  //x Long side of field
            new PIDController(2, .5, 0), //y Short side of field
            thetaController); // (2Pk,PI) constrains to 1 2pi/sec

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            Pigeon.getRotation2d(),
            positions,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );
    }

    public void zero(){
        frontLeftModule.zeroMotorPos();
        frontRightModule.zeroMotorPos();
        backLeftModule.zeroMotorPos();
        backRightModule.zeroMotorPos();
        
        frontLeftDrive.setSelectedSensorPosition(0);
        frontRightDrive.setSelectedSensorPosition(0);
        backLeftDrive.setSelectedSensorPosition(0);
        backRightDrive.setSelectedSensorPosition(0);

        if(realBot){
            frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        } else {
            frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        }

        positions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());
        positions[0].distanceMeters = frontLeftPos;

        positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
        positions[1].distanceMeters = frontRightPos;

        positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[2].distanceMeters = backLeftPos;

        positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
        positions[3].distanceMeters = backRightPos;

        poseEstimator.resetPosition(Pigeon.getRotation2d(),
                                    positions,
                                    new Pose2d());
    }

    public static void setPose(Pose2d pose, Rotation2d heading){
        frontLeftDrive.setSelectedSensorPosition(0);
        frontRightDrive.setSelectedSensorPosition(0);
        backLeftDrive.setSelectedSensorPosition(0);
        backRightDrive.setSelectedSensorPosition(0);

        Pigeon.zeroSensor(MathUtil.inputModulus(heading.getDegrees(), -180, 180));

        poseEstimator.resetPosition(heading,
                                    positions,
                                    pose);
    }
    
    public void setSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        states = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[3].angle.getRadians());
    }

    public void setModulePositions(double speed, double angle){
        frontLeftModule.set(speed, Math.toRadians(angle));
        frontRightModule.set(speed, Math.toRadians(angle));
        backLeftModule.set(speed, Math.toRadians(angle));
        backRightModule.set(speed, Math.toRadians(angle));
    }

    boolean firstLoop = true;
    boolean useApril = false;

    public void teleAction(TeleopCommander commander){
        updatePose();
        // Camera.switchPipe(useApril);

        if(commander.getPickUpObject()){
            useApril = false;
            firstLoop = true;
            // if(Camera.getV() == 1){
                chassisSpeeds = new ChassisSpeeds(
                    -(Camera.getA() - 5) * .4,
                    0,//-Camera.getX() * .175,
                    Camera.getX() * .15);
            // }
            setSwerveModuleStates(chassisSpeeds);
        } else if(commander.getDriveToScoring()){
            useApril = true;
            if(firstLoop){
                timer.reset();
                if(Camera.getV() == 1){
                    // setPose(Camera.getBotPose(), Camera.getBotPose().getRotation());
                } else {
                    setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), Rotation2d.fromDegrees(90));
                }
                firstLoop = false;
            }
            driveToPos(scoringPose, timer.get());
        } else {
            firstLoop = true;
            useApril = true;

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                commander.getForwardCommand(),
                commander.getStrafeCommand(),
                commander.getTurnCommand(),
                Pigeon.getRotation2d());

            setSwerveModuleStates(chassisSpeeds);
        }
    }

    public void disabled(){
    }

    public static Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public static void stopMotors(){
        frontLeftDrive.set(ControlMode.PercentOutput, 0);
        frontRightDrive.set(ControlMode.PercentOutput, 0);
        backLeftDrive.set(ControlMode.PercentOutput, 0);
        backRightDrive.set(ControlMode.PercentOutput, 0);
    }

    public void driveToPos(Pose2d pose, double time){
        setSwerveModuleStates(holonomicController.calculate(poseEstimator.getEstimatedPosition(), 
           new State(time, 1,1, pose, 1000),
           pose.getRotation()));
    }

    public void autonAction(AutonCommader autonCommader){
        updatePose();

        if(!autonCommader.getPickUpObject()){
            chassisSpeeds = holonomicController.calculate(poseEstimator.getEstimatedPosition(), 
                                                                autonCommader.getDesiredState(),
                                                                autonCommader.getDesiredState().poseMeters.getRotation());
        } else {
            if(Camera.getC().contains("person")){
                chassisSpeeds = new ChassisSpeeds(
                    -(Camera.getA() - 20) * .15,
                    0,//-Camera.getX() * .175,
                    -Camera.getX() * .15);
            } else {
                chassisSpeeds = new ChassisSpeeds(
                    0,
                    0,
                    0);
            }
        }

        setSwerveModuleStates(chassisSpeeds);

        SmartDashboard.putNumber("Estimated Theta", Rotation2d.fromDegrees(Pigeon.getAngle()).getDegrees());
        SmartDashboard.putNumber("Estimated X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Estimated Y", poseEstimator.getEstimatedPosition().getY());

        SmartDashboard.putNumber("Commanded Theta", autonCommader.getDesiredState().poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("Commanded X", autonCommader.getDesiredState().poseMeters.getX());
        SmartDashboard.putNumber("Commanded Y", autonCommader.getDesiredState().poseMeters.getY());
    }

    public void updatePose(){
        if(realBot){
            frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
            backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4I_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4I_L2.getWheelDiameter();
        } else {
            frontLeftPos = (frontLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            frontRightPos = (frontRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            backLeftPos = (backLeftDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
            backRightPos = (backRightDrive.getSelectedSensorPosition() / 2048) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * Math.PI * SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        }

        positions[0].angle = new Rotation2d(frontLeftModule.getSteerAngle());
        positions[0].distanceMeters = frontLeftPos;

        positions[1].angle = new Rotation2d(frontRightModule.getSteerAngle());
        positions[1].distanceMeters = frontRightPos;

        positions[2].angle = new Rotation2d(backLeftModule.getSteerAngle());
        positions[2].distanceMeters = backLeftPos;

        positions[3].angle = new Rotation2d(backRightModule.getSteerAngle());
        positions[3].distanceMeters = backRightPos;

        if (Camera.getV() == 1 && useApril) {
            // poseEstimator.addVisionMeasurement(Camera.getBotPose(), Timer.getFPGATimestamp());
        }

        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), Pigeon.getRotation2d(), positions);
    }

    // public void setBrakeMode(boolean brake){
    //     if(brake){
    //         frontLeftDrive.setNeutralMode(NeutralMode.Brake);
    //         frontRightDrive.setNeutralMode(NeutralMode.Brake);
    //         backLeftDrive.setNeutralMode(NeutralMode.Brake);
    //         backRightDrive.setNeutralMode(NeutralMode.Brake);
    
    //         frontLeftSteer.setNeutralMode(NeutralMode.Brake);
    //         frontRightSteer.setNeutralMode(NeutralMode.Brake);
    //         backLeftSteer.setNeutralMode(NeutralMode.Brake);
    //         backRightSteer.setNeutralMode(NeutralMode.Brake);
    //     } else {
    //         frontLeftDrive.setNeutralMode(NeutralMode.Coast);
    //         frontRightDrive.setNeutralMode(NeutralMode.Coast);
    //         backLeftDrive.setNeutralMode(NeutralMode.Coast);
    //         backRightDrive.setNeutralMode(NeutralMode.Coast);
    
    //         frontLeftSteer.setNeutralMode(NeutralMode.Coast);
    //         frontRightSteer.setNeutralMode(NeutralMode.Coast);
    //         backLeftSteer.setNeutralMode(NeutralMode.Coast);
    //         backRightSteer.setNeutralMode(NeutralMode.Coast);
    //     }
    // }
}
