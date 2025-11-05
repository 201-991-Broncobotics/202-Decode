package org.firstinspires.ftc.teamcode.util;


public class functions {

    /** Finds the smallest difference between two angles or gets the equivalent angle between -180 and
     * 180 when the currentAngle is 0 (and wrapAngle is 360).
     * A wrapAngle of 180 treats the targetAngle and the angle directly opposite of targetAngle the same
     *
     * @param currentAngle degrees
     * @param targetAngle degrees
     * @param wrapAngle Should be 360 unless you are doing a swerve module
     */
    public static double angleDifference(double currentAngle, double targetAngle, int wrapAngle) {
        double result1 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), wrapAngle * 100L) * 0.01;
        double result2 = Math.floorMod(Math.round((targetAngle - currentAngle) * 100), -wrapAngle * 100L) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }

    public static double normalizeAngle(double angle) {
        return angleDifference(-180, angle, 360) + 180;
    }

    public static double round(double input, int decimalPlaces) {
        return Math.round(input * (Math.pow(10, decimalPlaces))) / (Math.pow(10, decimalPlaces));
    }

    public static double capValue(double value, double limit) {
        if (Math.abs(value) > limit) return limit * Math.signum(value);
        else return value;
    }

    /**
     * If the absolute value of the input is less than the deadZone, this returns 0 though if it is greater,
     * this returns the difference between the deadZone and the input. Example: -0.5, -0.25, 0, 0, 0, 0.25, 0.5
     */
    public static double deadZoneFlattened(double value, double deadZone) {
        if (Math.abs(value) > deadZone) return value - Math.abs(deadZone) * Math.signum(value);
        else return 0.0;
    }

    public static double deadZoneNormalized(double value, double deadZone) {
        return deadZoneFlattened(value, deadZone) * Math.abs(1 / (1 - deadZone));
    }

    public static void Sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            // Wait the set amount of time in milliseconds
        }
    }

    public static double minMaxValue(double min, double max, double value) {
        if (value > max) return max;
        else return Math.max(value, min);
    }

}
