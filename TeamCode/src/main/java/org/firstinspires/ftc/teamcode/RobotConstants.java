package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.util.subsystem.StateSubsystemBase;

/**
 * Contains all robot-wide constants for the robot.
 */
public class RobotConstants {

    /**
     * Constants related to the robot's drivetrain.
     */
    public static class DriveConstants {
        /**
         * Number of encoder ticks per motor revolution.
         */
        public static final double TICKS_PER_REV = 2000;

        /**
         * Gear ratio from motor to wheel (output/input).
         */
        public static final double GEAR_RATIO = 1;

        /**
         * Radius of the wheels in meters.
         */
        public static final double WHEEL_RADIUS = 0.024;

        /**
         * Distance between left and right wheels (track width) in meters.
         */
        public static final double WHEELS_DISTANCE = 0.207;

        /**
         * Feedforward constant (Ks)
         */
        public static final double Ks = 0;

        /**
         * Orientation of the REV Hub logo on the robot.
         */
        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;

        /**
         * Orientation of the USB ports on the REV Hub.
         */
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        /**
         * Predefined drive speed modes with a modifier for scaling motor power.
         */
        public enum DriveSpeed {
            NORMAL(1),
            SLOW(0.65);

            private final double modifier;

            DriveSpeed(double modifier) {
                this.modifier = modifier;
            }

            public double getSpeedModifier() {
                return modifier;
            }
        }
    }

    @Configurable
    public static class StorageConstants {
        @Sorter(sort = 1) public static double kP = 0.03;
        @Sorter(sort = 2) public static double kI = 0.0001;
        @Sorter(sort = 3) public static double kD = 0.002;

        public enum StorageState implements StateSubsystemBase.StateBase<Double> {
            SHOOTER(38),
            INTAKE(0);

            private final double deg;

            StorageState(double deg){
                this.deg = deg;
            }

            @Override
            public Double getUnit() {
                return this.deg;
            }
        }
    }

    public static class GateConstants {
        public enum GateState implements StateSubsystemBase.StateBase<Double> {
            ZERO(0),
            PUSH(120);

            private final double deg;

            GateState(double deg) {
                this.deg = deg;
            }

            @Override
            public Double getUnit() {
                return this.deg;
            }
        }
    }

    public static class ShooterConstants {
        public enum ShooterState implements StateSubsystemBase.StateBase<Double> {
            SHOOT(0.75),
            OFF(0),
            IDLE(0.4);

            private final double power;

            ShooterState(double power) {
                this.power = power;
            }

            @Override
            public Double getUnit() {
                return this.power;
            }
        }
    }
}
