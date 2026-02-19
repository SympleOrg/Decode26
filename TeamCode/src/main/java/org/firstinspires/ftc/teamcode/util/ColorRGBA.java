package org.firstinspires.ftc.teamcode.util;

public class ColorRGBA {
    private double red;
    private double green;
    private double blue;
    private double alpha;

    public ColorRGBA(double red, double green, double blue, double alpha) {
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.alpha = alpha;
    }

    public double red() {
        return red;
    }

    public double green() {
        return green;
    }

    public double blue() {
        return blue;
    }

    public double alpha() {
        return alpha;
    }
}
