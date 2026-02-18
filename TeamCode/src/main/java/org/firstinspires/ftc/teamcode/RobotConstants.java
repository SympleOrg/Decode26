package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Contains all robot-wide constants for the robot.
 */
public class RobotConstants {

    @Configurable
    public static class FieldConstants {
        public static Pose RED_GOAL_POSE = new Pose(0, 0);
        public static Pose BLUE_GOAL_POSE = new Pose();

        public static double GOAL_HEIGHT = 1;
    }

    public static class AutoConstants {
        public static Pose RED_GOAL_POSE = new Pose(111.123, 135.671);
    }

    /**
     * Constants related to the robot's drivetrain.
     */
    @Configurable
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

    public static class StorageConstants {
        @Sorter(sort = -1)
        public static long TRANSFER_TIME = 250;

        @Sorter(sort = 0)
        public static double LOWER_STORAGE_SERVO_ANGLE = 0;
        @Sorter(sort = 1)
        public static double HIGHER_STORAGE_SERVO_ANGLE = 90;

        @Sorter(sort = 2)
        public static double LOWER_SHOOTER_SERVO_ANGLE = 0;
        @Sorter(sort = 3)
        public static double HIGHER_SHOOTER_SERVO_ANGLE = 90;

        public static class PurpleBall {
            @Sorter(sort = 0)
            public static double MIN_RED = 0;
            @Sorter(sort = 1)
            public static double MIN_GREEN = 0;
            @Sorter(sort = 2)
            public static double MIN_BLUE = 0;
        }

        public static class GreenBall {
            @Sorter(sort = 0)
            public static double MIN_RED = 0;
            @Sorter(sort = 1)
            public static double MIN_GREEN = 0;
            @Sorter(sort = 2)
            public static double MIN_BLUE = 0;
        }
    }

    @Configurable
    public static class TurretConstants {
        public static Pose TURRET_OFFSET = new Pose(0, 0);
        public static double SHOOTER_HEIGHT = 0.3;

        public static class Shooter {
            @Sorter(sort = -6)
            public static double HOOD_MIN_ANGLE = 10;
            @Sorter(sort = -5)
            public static double HOOD_MAX_ANGLE = 40;
            @Sorter(sort = -4)
            public static double HOOD_GEAR_RATIO = 40 / 355f;

            @Sorter(sort = -3)
            public static double SHOOT_OFFSET_VEL = -50;
            @Sorter(sort = -2)
            public static long SHOOTER_DELAY = 200;

            @Sorter(sort = -1)
            public static double WHEEL_RADIUS = 0.048;
            @Sorter(sort = 0)
            public static double SHOOTER_GEAR_RATIO = 1;

            @Sorter(sort = 1)
            public static double Kp = 0.5;
            @Sorter(sort = 2)
            public static double Ki = 0.05;
            @Sorter(sort = 3)
            public static double Kd = 0;
            @Sorter(sort = 4)
            public static double Ks = 230;
            @Sorter(sort = 5)
            public static double Kv = 1.3;
            @Sorter(sort = 6)
            public static double Ka = 0;
        }

        public static class Turret {
            @Sorter(sort = -2)
            public static double MIN_ANGLE = -200;
            @Sorter(sort = -1)
            public static double MAX_ANGLE = 200;

            @Sorter(sort = 0)
            public static double Kp = 0;
            @Sorter(sort = 1)
            public static double Ki = 0;
            @Sorter(sort = 2)
            public static double Kd = 0;
        }
    }
}
