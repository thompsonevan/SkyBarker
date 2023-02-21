package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    public TalonFX getDriveMotor();
    public TalonFX getSteerMotor();
    public AbsoluteEncoder getEncoder();
    public void zeroMotorPos();
}
