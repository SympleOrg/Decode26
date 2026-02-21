package org.firstinspires.ftc.teamcode.opMode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoRobotController;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

@Autonomous(name = "Red auto", group = "auto")
public class RedAutoOpMode extends SympleCommandOpMode {
    @Override
    public void initialize() {
        this.robotController = new AutoRobotController.Builder()
                .initializeDefaults(this)
                .setTeamColor(TeamColor.RED)
                .setStartingPose(AutoRobotController.StartingPose.CLOSE_GOAL)
                .build();
    }
}
