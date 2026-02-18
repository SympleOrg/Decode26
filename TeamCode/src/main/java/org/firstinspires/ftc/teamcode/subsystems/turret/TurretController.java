package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.RobotConstants.FieldConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.TurretConstants;
import org.firstinspires.ftc.teamcode.util.TeamColor;

public class TurretController {
    private final TurretSubsystem turretSubsystem;

    private final PIDController turrentPidController;

    // In Radians
    private double hoodAngle = 0;

    protected TurretController(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;

        this.turrentPidController = new PIDController(TurretConstants.Turret.Kp, TurretConstants.Turret.Ki, TurretConstants.Turret.Kd);
        this.turrentPidController.setTolerance(1);
        this.turrentPidController.reset();
    }

    public void update(Pose robotPose, TeamColor teamColor) {
        Pose goalPose = teamColor == TeamColor.RED ? FieldConstants.RED_GOAL_POSE : FieldConstants.BLUE_GOAL_POSE;

        this.updateTurret(robotPose, goalPose);
        this.updateHood(robotPose, goalPose);
        this.updateFlywheel(this.hoodAngle);
    }

    // Turret
    private void updateTurret(Pose robotPose, Pose goalPose) {
        Pose turretPose = this.getTurretFieldPose(robotPose);

        Pose poseDifference = goalPose.minus(turretPose);
        double requiredAngle = Math.atan2(poseDifference.getY(), poseDifference.getX());

        this.turretSubsystem.getTelemetry().addData("Turret Angle", this.turretSubsystem.getCurrentTurretAngle());
        this.turretSubsystem.getTelemetry().addData("Required Turret Angle", requiredAngle);

        double angleRange = TurretConstants.Turret.MAX_ANGLE - TurretConstants.Turret.MIN_ANGLE;

        if (requiredAngle > TurretConstants.Turret.MAX_ANGLE) requiredAngle -= angleRange;
        if (requiredAngle < TurretConstants.Turret.MIN_ANGLE) requiredAngle += angleRange;

        this.setTurretAngle(requiredAngle);
        this.adjustTurretAngle();
    }

    public void setTurretAngle(double requiredAngle) {
        this.turrentPidController.setSetPoint(requiredAngle);
    }

    private void adjustTurretAngle() {
        double power = this.turrentPidController.calculate(this.turretSubsystem.getCurrentTurretAngle());

        this.turretSubsystem.moveTurret(power);
    }

    // Hood
    private void updateHood(Pose robotPose, Pose goalPose) {
        Pose turretPose = this.getTurretFieldPose(robotPose);

        double requiredAngle = calculateHoodAngle(goalPose.distanceFrom(turretPose));
        requiredAngle = MathUtils.clamp(requiredAngle, Math.toRadians(TurretConstants.Shooter.HOOD_MIN_ANGLE), Math.toRadians(TurretConstants.Shooter.HOOD_MAX_ANGLE));
        this.hoodAngle = requiredAngle;

        this.turretSubsystem.getTelemetry().addData("Required Hood Angle", Math.toDegrees(requiredAngle));

        this.turretSubsystem.moveHood(Math.toDegrees(requiredAngle) / TurretConstants.Shooter.HOOD_GEAR_RATIO);
    }

    private double calculateHoodAngle(double distance) {
        double travelHeight = FieldConstants.GOAL_HEIGHT - TurretConstants.SHOOTER_HEIGHT;
        return Math.atan((travelHeight * 2) / distance);
    }

    // Flywheel
    private void updateFlywheel(double shootAngle) {
        double requiredVelocity = this.calculateFlywheelVelocity(shootAngle);

        this.turretSubsystem.getTelemetry().addData("Flywheel Velocity", this.turretSubsystem.getFlywheelVelocity());
        this.turretSubsystem.getTelemetry().addData("Required Flywheel Velocity", requiredVelocity > 0 ? requiredVelocity : "Impossible shoot location!");

        this.turretSubsystem.setVelocity(requiredVelocity > 0 ? requiredVelocity : 0);
    }

    private double calculateFlywheelVelocity(double shootAngle) {
        final double gravity = 9.81;

        double travelHeight = FieldConstants.GOAL_HEIGHT - TurretConstants.SHOOTER_HEIGHT;

        double poweredProduct = (2 * travelHeight * gravity) / (Math.sin(2 * shootAngle));

        if (poweredProduct < 0) return -1;

        return Math.sqrt(poweredProduct);
    }

    // Util
    private Pose getTurretFieldPose(Pose robotPose) {
        return robotPose.plus(TurretConstants.TURRET_OFFSET.rotate(robotPose.getHeading(), false));
    }
}
