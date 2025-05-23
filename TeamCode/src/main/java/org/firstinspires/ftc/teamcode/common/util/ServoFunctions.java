package org.firstinspires.ftc.teamcode.common.util;

public class ServoFunctions {
    public static double degreesToServoPosition(double degrees, double servoRange) {
        return degrees / servoRange;
    }

    public static double servoPositionToDegrees(double position, double servoRange) {
        return position * servoRange;
    }

    public static double clampServoPosition(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }

    public static double clampAngleDegrees(double degrees, double minAngle, double maxAngle) {
        return Math.max(minAngle, Math.min(maxAngle, degrees));
    }
}
