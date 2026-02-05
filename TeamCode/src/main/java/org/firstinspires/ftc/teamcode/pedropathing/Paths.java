package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {
    public static PathChain createRedPath(Follower follower) {
        return follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(111.123, 135.671),
                                new Pose(60.000, 135.671)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
    }
}
