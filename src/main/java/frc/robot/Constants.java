// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

// Swerve Bot Constants
public final class Constants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4445;

    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

    public static final double MAX_VOLTAGE = 12.0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6380.0 / 60.0 *
        SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    
    public static final int DRIVETRAIN_PIGEON_ID = 50;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(21.97265625);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 17;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(58.71093750000001);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 14;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(309.27886962890625);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 18;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(212.2890625);

    public static final double INTAKE_PACKAGE_POSITION = 0;
    public static final double INTAKE_STATION_POSITION = 45;
    public static final double INTAKE_COLLECT_POSITION = 90;
    public static final double INTAKE_SPEED = .45;
    
    //Arm CAN ID
    public static final int SHOULDER = 13;
    public static final int EXTENSION = 14;
    public static final int ELBOW = 52;
    public static final int SHOULDER_ENCODER = 22;
    public static final int ELBOW_ENCODER = 100;

    //Arm pid motion magic gains
    public static final double SHOULDER_MOTOR_kF = 0;
    public static final double SHOULDER_MOTOR_kP = 0.4;
    public static final double SHOULDER_MOTOR_kI = 0;
    public static final double SHOULDER_MOTOR_kD = 0;

    public static final double EXTENSION_MOTOR_kF = 0;
    public static final double EXTENSION_MOTOR_kP = 0.4;
    public static final double EXTENSION_MOTOR_kI = 0;
    public static final double EXTENSION_MOTOR_kD = 0;

    public static final double ELBOW_MOTOR_kF = 0;
    public static final double ELBOW_MOTOR_kP = 0.2;
    public static final double ELBOW_MOTOR_kI = 0;
    public static final double ELBOW_MOTOR_kD = 0;

    //Arm motion magic parameters
    public static final double SHOULDER_CRUISEVELOCITY = 16000 * 6;
    public static final double SHOULDER_ACCEL = 6000 * 6;

    public static final double EXTENSION_CRUISEVELOCITY = 16000 * 6;
    public static final double EXTENSION_ACCEL = 6000 * 6;

    public static final double ELBOW_CRUIESVELOCITY = 1600 * 6;
    public static final double ELBOW_ACCEL = 600 * 6;

    //Arm pid slots
    public static final int SHOULDER_PID_SLOT = 0;
    public static final int SHOULDER_K_PID_LOOP_IDX = 0;

    public static final int EXTENSION_PID_SLOT = 0;
    public static final int EXTENSION_K_PID_LOOP_IDX = 0;

    public static final int ELBOW_PID_SLOT = 0;
    public static final int ELBOW_K_PID_LOOP_IDX = 0;

    public static final double FALCON500_TICKS_PER_REV = 2048;
    public static final double SHOULDER_DEGREES_TO_TICKS = -FALCON500_TICKS_PER_REV * 1.3786;
    public static final double ELBOW_DEGREES_TO_TICKS = -FALCON500_TICKS_PER_REV * 1;

    //Arm motion magic set positions
    public static final double SHOULDER_TARGET_POSITION_LOW = 90 * SHOULDER_DEGREES_TO_TICKS; // In degrees
    public static final double EXTENSION_TARGET_POSITION_LOW = 0 * FALCON500_TICKS_PER_REV; // In Revolutions
    public static final double ELBOW_TARGET_POSITION_LOW = 1;

    public static final double SHOULDER_TARGET_POSITION_MIDDLE = 80 * SHOULDER_DEGREES_TO_TICKS;
    public static final double EXTENSION_TARGET_POSITION_MIDDLE = 0 * FALCON500_TICKS_PER_REV;
    public static final double ELBOW_TARGET_POSITION_MIDDLE = 2;

    public static final double SHOULDER_TARGET_POSITION_HIGH = 45 * SHOULDER_DEGREES_TO_TICKS;
    public static final double EXTENSION_TARGET_POSITION_HIGH = 50 * FALCON500_TICKS_PER_REV;
    public static final double ELBOW_TARGET_POSITION_HIGH = 2;

    public static final double SHOULDER_TARGET_POSITION_PACKAGE = 0 * SHOULDER_DEGREES_TO_TICKS;
    public static final double EXTENSION_TARGET_POSITION_PACKAGE = 0 * FALCON500_TICKS_PER_REV;
    public static final double ELBOW_TARGET_POSITION_PACKAGE = 0;

    public static final int ARM_TIMEOUT = 200;
}


// Practice Bot Constants
// public final class Constants {
//     public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4445;

//     public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

//     public static final double MAX_VOLTAGE = 12.0;

//     public static final double MAX_VELOCITY_METERS_PER_SECOND =
//         6380.0 / 60.0 *
//         SdsModuleConfigurations.MK4_L2.getDriveReduction() *
//         SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

//     public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
//         Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

//     public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
//         new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
//         new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
//         new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
//         new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));
    
//     public static final int DRIVETRAIN_PIGEON_ID = 50;

//     public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
//     public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
//     public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13;
//     public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(21.97265625);

//     public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
//     public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
//     public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 17;
//     public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(58.71093750000001);

//     public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 2;
//     public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
//     public static final int BACK_LEFT_MODULE_STEER_ENCODER = 14;
//     public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(309.27886962890625);

//     public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
//     public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
//     public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 18;
//     public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(212.2890625);

//     //Arm CAN ID
//     public static final int SHOULDER = 13;
//     public static final int EXTENSION = 14;
//     public static final int ELBOW = 52;

//     //Arm pid motion magic gains
//     public static final double SHOULDER_MOTOR_kF = 0;
//     public static final double SHOULDER_MOTOR_kP = 0.4;
//     public static final double SHOULDER_MOTOR_kI = 0;
//     public static final double SHOULDER_MOTOR_kD = 0;

//     public static final double EXTENSION_MOTOR_kF = 0;
//     public static final double EXTENSION_MOTOR_kP = 0.4;
//     public static final double EXTENSION_MOTOR_kI = 0;
//     public static final double EXTENSION_MOTOR_kD = 0;

//     public static final double ELBOW_MOTOR_kF = 0;
//     public static final double ELBOW_MOTOR_kP = 0.2;
//     public static final double ELBOW_MOTOR_kI = 0;
//     public static final double ELBOW_MOTOR_kD = 0;


//     //Arm motion magic parameters
//     public static final double SHOULDER_CRUISEVELOCITY = 16000 * 6;
//     public static final double SHOULDER_ACCEL = 6000 * 6;

//     public static final double EXTENSION_CRUISEVELOCITY = 16000 * 6;
//     public static final double EXTENSION_ACCEL = 6000 * 6;

//     public static final double ELBOW_CRUIESVELOCITY = 1600 * 6;
//     public static final double ELBOW_ACCEL = 600 * 6;


//     //Arm pid slots
//     public static final int SHOULDER_PID_SLOT = 0;
//     public static final int SHOULDER_K_PID_LOOP_IDX = 0;

//     public static final int EXTENSION_PID_SLOT = 0;
//     public static final int EXTENSION_K_PID_LOOP_IDX = 0;

//     public static final int ELBOW_PID_SLOT = 0;
//     public static final int ELBOW_K_PID_LOOP_IDX = 0;

//     public static final double FALCON500_TICKS_PER_REV = 2048;
//     public static final double SHOULDER_DEGREES_TO_TICKS = -FALCON500_TICKS_PER_REV * 1.3786;

//     //Arm motion magic set positions
//     public static final double SHOULDER_TARGET_POSITION_LOW = 90 * SHOULDER_DEGREES_TO_TICKS; // In degrees
//     public static final double EXTENSION_TARGET_POSITION_LOW = 20 * FALCON500_TICKS_PER_REV; // In Revolutions
//     public static final double ELBOW_TARGET_POSITION_LOW = 1;

//     public static final double SHOULDER_TARGET_POSITION_MIDDLE = -90 * SHOULDER_DEGREES_TO_TICKS;
//     public static final double EXTENSION_TARGET_POSITION_MIDDLE = 50 * FALCON500_TICKS_PER_REV;
//     public static final double ELBOW_TARGET_POSITION_MIDDLE = 2;

//     public static final double SHOULDER_TARGET_POSITION_HIGH = -90 * SHOULDER_DEGREES_TO_TICKS;
//     public static final double EXTENSION_TARGET_POSITION_HIGH = 50 * FALCON500_TICKS_PER_REV;
//     public static final double ELBOW_TARGET_POSITION_HIGH = 2;

//     public static final double SHOULDER_TARGET_POSITION_PACKAGE = 0 * SHOULDER_DEGREES_TO_TICKS;
//     public static final double EXTENSION_TARGET_POSITION_PACKAGE = 0 * FALCON500_TICKS_PER_REV;
//     public static final double ELBOW_TARGET_POSITION_PACKAGE = 2;

//     public static final int ARM_TIMEOUT = 200;

// }
