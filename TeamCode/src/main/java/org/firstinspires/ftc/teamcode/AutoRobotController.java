package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedropathing.Trajectories;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.storage.StorageSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public class AutoRobotController extends RobotControllerBase {
    private final MecanumDriveSubsystem mecanumDriveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final StorageSubsystem storageSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    private final TeamColor teamColor;

    private final Trajectories trajectories;

    private final StartingPose startingPose;
    private Command autoCommand;

    public AutoRobotController(HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, String logFilePrefix, boolean logData, TeamColor teamColor, StartingPose startingPose) {
        super(hMap, telemetry, driverController, actionController, logFilePrefix, logData);

        this.mecanumDriveSubsystem = new MecanumDriveSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.intakeSubsystem = new IntakeSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.storageSubsystem = new StorageSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.turretSubsystem = new TurretSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());
        this.elevatorSubsystem = new ElevatorSubsystem(this.getHardwareMap(), this.getTelemetry(), this.getDataLogger());

        this.teamColor = teamColor;
        this.startingPose = startingPose;

        this.trajectories = new Trajectories(
                this.mecanumDriveSubsystem,
                this.intakeSubsystem,
                this.storageSubsystem,
                this.turretSubsystem,
                this.elevatorSubsystem
        );
    }

    @Override
    public void createKeyBindings() {

    }

    @Override
    public void initialize() {
        if (this.teamColor == TeamColor.RED && this.startingPose == StartingPose.CLOSE_GOAL) {
            getPathFollower().setStartingPose(RobotConstants.AutoConstants.RED_CLOSE_GOAL_POSE);
            this.autoCommand = this.trajectories.createCloseRedTrajectory();
        }

        if (this.teamColor == TeamColor.RED && this.startingPose == StartingPose.FAR_GOAL) {
            getPathFollower().setStartingPose(RobotConstants.AutoConstants.RED_FAR_GOAL_POSE);
            this.autoCommand = this.trajectories.createFarRedTrajectory();
        }

        if (this.teamColor == TeamColor.BLUE && this.startingPose == StartingPose.CLOSE_GOAL) {
            getPathFollower().setStartingPose(RobotConstants.AutoConstants.BLUE_CLOSE_GOAL_POSE);
            this.autoCommand = this.trajectories.createCloseBlueTrajectory();
        }

        if (this.teamColor == TeamColor.BLUE && this.startingPose == StartingPose.FAR_GOAL) {
            getPathFollower().setStartingPose(RobotConstants.AutoConstants.BLUE_FAR_GOAL_POSE);
            this.autoCommand = this.trajectories.createFarBlueTrajectory();
        }
    }

    @Override
    public void initializeLoop() {

    }

    @Override
    public void postInitialize() {
        this.autoCommand.schedule();
    }

    @Override
    public void run() {

    }

    @Override
    public void postRun() {

    }

    public enum StartingPose {
        CLOSE_GOAL,
        FAR_GOAL
    }

    public static class Builder extends RobotControllerBase.Builder {
        private TeamColor teamColor;
        private StartingPose startingPose;

        public Builder() {
            this.logFilePrefix = "Auto";
        }

        @Override
        public Builder initializeDefaults(SympleCommandOpMode opMode) {
            super.initializeDefaults(opMode);
            return this;
        }

        public Builder setTeamColor(TeamColor teamColor) {
            this.teamColor = teamColor;
            return this;
        }

        public Builder setStartingPose(StartingPose startingPose) {
            this.startingPose = startingPose;
            return this;
        }

        @Override
        public AutoRobotController build() {
            if (teamColor == null) throw new IllegalArgumentException("No set team color!");
            return new AutoRobotController(this.hardwareMap, this.telemetry, this.driverController, this.actionController, this.logFilePrefix, this.logData, this.teamColor, this.startingPose);
        }
    }
}
