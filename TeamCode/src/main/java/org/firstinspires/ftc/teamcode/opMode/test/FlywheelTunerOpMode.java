package org.firstinspires.ftc.teamcode.opMode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.maps.MotorMap;

import java.util.concurrent.ThreadLocalRandom;

import kotlin.random.Random;

@TeleOp(name = "Flywheel Tuner", group = "test")
@Configurable
public class FlywheelTunerOpMode extends CommandOpMode {
    @Sorter(sort = -3)
    public static MotorMap motorId;
    @Sorter(sort = -2)
    public static long MIN_VEL = 0;
    @Sorter(sort = -1)
    public static long MAX_VEL = 0;
    @Sorter(sort = 0)
    public static long INTERVAL_TIME = 3000;
    @Sorter(sort = 1)
    public static double Kp = 0;
    @Sorter(sort = 2)
    public static double Ki = 0;
    @Sorter(sort = 3)
    public static double Kd = 0;
    @Sorter(sort = 4)
    public static double Ks = 0;
    @Sorter(sort = 5)
    public static double Kv = 0;
    @Sorter(sort = 6)
    public static double Ka = 0;

    private MotorEx motorEx;
    private TelemetryManager telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private long targetVelocity = MAX_VEL;
    private ElapsedTime timer;

    @Override
    public void initialize() {

    }

    private void postInitialize() {
        if (motorId == null) throw new IllegalArgumentException("MotorId cannot be null!");

        this.motorEx = new MotorEx(hardwareMap, motorId.getId(), motorId.getTicksPerRev(), motorId.getMaxRPM());
        this.motorEx.setRunMode(Motor.RunMode.VelocityControl);
        this.motorEx.setVeloCoefficients(Kp, Ki, Kd);
        this.motorEx.setFeedforwardCoefficients(Ks, Kv, Ka);

        this.timer = new ElapsedTime();
    }


    private void initializeLoop() {

    }

    @Override
    public void run() {
        super.run();

        if (this.timer.milliseconds() > INTERVAL_TIME) {
            this.timer.reset();

            long randomVel = ThreadLocalRandom.current().nextLong(MIN_VEL,MAX_VEL);

            this.targetVelocity = randomVel - (randomVel % 20);
        }

        this.motorEx.setVeloCoefficients(Kp, Ki, Kd);
        this.motorEx.setFeedforwardCoefficients(Ks, Kv, Ka);
        this.motorEx.setVelocity(this.targetVelocity);

        telemetry.addData("Current Velocity", this.motorEx.getCorrectedVelocity());
        telemetry.addData("Target Velocity", this.targetVelocity);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        this.initialize();

        // runs when in init mode
        while (this.opModeInInit() && !this.isStopRequested()) {
            initializeLoop();
        }

        this.waitForStart();

        postInitialize();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            this.run();
        }

        this.reset();
    }
}