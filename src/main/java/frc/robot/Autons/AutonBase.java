package frc.robot.Autons;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;

public abstract class AutonBase {
    public State desState;
    public Rotation2d targetTheta;
    public Timer timer;
    public PathPlannerState initalState;

    public abstract void runAuto();
    public abstract void reset();
}
