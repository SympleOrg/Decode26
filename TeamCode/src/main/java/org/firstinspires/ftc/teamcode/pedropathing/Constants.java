package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.maps.MotorMap;
import org.firstinspires.ftc.teamcode.maps.SensorMap;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Constants {

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(62.133539031492184)
            .yVelocity(44.498033906248956)
            .rightFrontMotorName(MotorMap.LEG_FRONT_RIGHT.getId())
            .rightRearMotorName(MotorMap.LEG_BACK_RIGHT.getId())
            .leftRearMotorName(MotorMap.LEG_BACK_LEFT.getId())
            .leftFrontMotorName(MotorMap.LEG_FRONT_LEFT.getId())
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches((MathUtil.metersToInch(Math.PI * 2) * 0.024) / 2000)
            .strafeTicksToInches((MathUtil.metersToInch(Math.PI * 2) * 0.024) / 2000)
            .turnTicksToInches((MathUtil.metersToInch(Math.PI * 2) * 0.024) / 2000)
            .leftPodY(MathUtil.metersToInch(0.103))
            .rightPodY(MathUtil.metersToInch(-0.103))
            .strafePodX(MathUtil.metersToInch(-0.165))
            .leftEncoder_HardwareMapName(SensorMap.DEAD_WHEEL_LEFT.getId())
            .rightEncoder_HardwareMapName(SensorMap.DEAD_WHEEL_RIGHT.getId())
            .strafeEncoder_HardwareMapName(SensorMap.DEAD_WHEEL_BACK.getId())
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RobotConstants.DriveConstants.LOGO_FACING_DIRECTION, RobotConstants.DriveConstants.USB_FACING_DIRECTION));

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.4)
            .forwardZeroPowerAcceleration(-26.35075469661182)
            .forwardZeroPowerAcceleration(-57.845484293764414);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}