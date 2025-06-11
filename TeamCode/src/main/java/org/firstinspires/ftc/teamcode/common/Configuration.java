package org.firstinspires.ftc.teamcode.common;

@com.acmerobotics.dashboard.config.Config
public class Configuration {
    public static double intakeSlides_kP = 0.01, intakeSlides_kI = 0.0, intakeSlides_kD = 0.0, intakeSlides_kF = 0.0, intakeSlides_tolerance = 10.0, intakeSlides_ticksPerCM = 0.0;

    public static double turret_kP = 0.01, turret_kI = 0.0, turret_kD = 0.0, turret_kF = 0.0, turret_tolerance = 10.0;

    public static double clipperDrive_kP = 0.01, clipperDrive_kI = 0.0, clipperDrive_kD = 0.0002, clipperDrive_kF = 0.0, clipperDrive_tolerance = 10.0, clipperDrive_ticksPerCM = 37.135;

    public static double depositSlides_kP = 0.01, depositSlides_kI = 0.0, depositSlides_kD = 0.0, depositSlides_kF = 0.0, depositSlides_tolerance = 10.0, depositSlides_ticksPerCM = 0.0;

    public static double clipMagOffset = 6.16, clipDriveClipPosition = 3.4;

    public static double clipperClipPosition = 0.0, clipperSetPosition = 0.4;

    public static double clipMagUp = 0.0, clipMagDown = 0.64;
}
