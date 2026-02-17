package org.firstinspires.ftc.teamcode.maps;

/**
 * Contains all servos
 * <pre>
 *  {@code
 *  CLAW("claw"),
 *  DRONE("drone");
 *  }
 * </pre>
 **/
public enum ServoMap {
    GATE("gate"),
    HOOD("hood");

    private final String id;

    ServoMap(String id) {
        this.id = id;
    }

    public String getId() {
        return id;
    }
}
