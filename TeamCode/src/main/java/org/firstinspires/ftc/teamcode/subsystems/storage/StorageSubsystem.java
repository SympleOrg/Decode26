package org.firstinspires.ftc.teamcode.subsystems.storage;

import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotConstants.StorageConstants;
import org.firstinspires.ftc.teamcode.maps.SensorMap;
import org.firstinspires.ftc.teamcode.maps.ServoMap;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.SympleServo;
import org.firstinspires.ftc.teamcode.util.subsystem.LoggerSubsystem;

public class StorageSubsystem extends SubsystemBase implements LoggerSubsystem {
    private final SympleServo shooterServo;
    private final SympleServo storageServo;
    private final NormalizedColorSensor shooterColorSensor;
    private final NormalizedColorSensor storageColorSensor;

    private final Timer indexerTimer;
    private final BallColor[] indexer;

    private final JoinedTelemetry telemetry;
    private final DataLogger dataLogger;

    public StorageSubsystem(HardwareMap hardwareMap, JoinedTelemetry telemetry, DataLogger dataLogger) {
        this.shooterServo = new SympleServo(hardwareMap, ServoMap.SHOOTER.getId(), 0, 300);
        this.storageServo = new SympleServo(hardwareMap, ServoMap.SHOOTER.getId(), 0, 300);
        this.shooterColorSensor = hardwareMap.get(NormalizedColorSensor.class, SensorMap.SHOOTER_COLOR.getId());
        this.storageColorSensor = hardwareMap.get(NormalizedColorSensor.class, SensorMap.STORAGE_COLOR.getId());

        this.indexerTimer = new Timer();
        this.indexer = new BallColor[] {BallColor.NONE, BallColor.NONE, BallColor.NONE};

        this.dataLogger = dataLogger;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        BallColor storageBall = this.getBallColor(storageColorSensor.getNormalizedColors());
        BallColor shooterBall = this.getBallColor(shooterColorSensor.getNormalizedColors());

        // Run loop to transfer ball from intake to storage
        if (indexerTimer.getElapsedTime() > StorageConstants.TRANSFER_TIME * 2) {
            this.moveStorageServo(StorageConstants.LOWER_STORAGE_SERVO_ANGLE);
        } else if (indexerTimer.getElapsedTime() > StorageConstants.TRANSFER_TIME) {
            indexer[1] = indexer[0];
            indexer[0] = BallColor.NONE;
        }

        if (shooterBall != BallColor.NONE) {
            this.moveShooterServo(StorageConstants.LOWER_SHOOTER_SERVO_ANGLE);
            indexer[2] = indexer[1];
            indexer[1] = BallColor.NONE;
        }

        if (storageBall != BallColor.NONE && indexer[0] == BallColor.NONE) {
            indexer[0] = storageBall;
        }

        if (indexer[1] == BallColor.NONE && indexer[0] != BallColor.NONE && indexerTimer.getElapsedTime() > StorageConstants.TRANSFER_TIME * 2) {
            this.moveStorageServo(StorageConstants.HIGHER_STORAGE_SERVO_ANGLE);
            indexerTimer.resetTimer();
        }

        if (indexer[2] == BallColor.NONE && indexer[1] != BallColor.NONE) {
            this.moveShooterServo(StorageConstants.HIGHER_SHOOTER_SERVO_ANGLE);
        }

        telemetry.addData("Indexer Timer", indexerTimer.getElapsedTime());
        telemetry.addData("Storage Ball Type (Sensor)", storageBall.name());
        telemetry.addData("Shooter Ball Type (Sensor)", shooterBall.name());
        telemetry.addData("Indexer [0] Ball Type", indexer[0].name());
        telemetry.addData("Indexer [1] Ball Type", indexer[1].name());
        telemetry.addData("Indexer [2] Ball Type", indexer[2].name());
    }

    public void setBall(int index, BallColor ballColor) {
        if (index < 0 || index >= indexer.length) throw new RuntimeException(String.format("Index must be between 0 and %s", indexer.length - 1));
        this.indexer[index] = ballColor;
    }

    private void moveShooterServo(double deg) {
        this.shooterServo.set(deg);
    }

    private void moveStorageServo(double deg) {
        this.storageServo.set(deg);
    }

    private BallColor getBallColor(NormalizedRGBA color) {
        if (
                color.red >= StorageConstants.GreenBall.MIN_RED / 255f
                && color.green >= StorageConstants.GreenBall.MIN_GREEN / 255f
                && color.blue >= StorageConstants.GreenBall.MIN_BLUE / 255f
        ) return BallColor.GREEN;

        if (
                color.red >= StorageConstants.PurpleBall.MIN_RED / 255f
                && color.green >= StorageConstants.PurpleBall.MIN_GREEN / 255f
                && color.blue >= StorageConstants.PurpleBall.MIN_BLUE / 255f
        ) return BallColor.PURPLE;

        return BallColor.NONE;
    }

    @Override
    public JoinedTelemetry getTelemetry() {
        return this.telemetry;
    }

    @Override
    public DataLogger getDataLogger() {
        return this.dataLogger;
    }
}
