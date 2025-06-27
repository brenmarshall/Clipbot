package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.clipper.ManualClipperDriveCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.clipper.SetClipMagGripperPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.clipper.SetClipMagPivotPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.clipper.TestClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.util.ExtendedGamepadButton;
import org.firstinspires.ftc.teamcode.common.util.ExtendedGamepadEx;
import org.firstinspires.ftc.teamcode.common.util.ExtendedGamepadKeys;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test", group = "TeleOp")
public class Test extends LinearOpMode {

    private Bot bot;
    private Intake intake;
    private Clipper clipper;

    private ExtendedGamepadEx driverGamepad;
    private ExtendedGamepadEx tuningGamepad;

    private MultipleTelemetry telem;

    @Override
    public void runOpMode() {

        CommandScheduler.getInstance().reset();

        telem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driverGamepad = new ExtendedGamepadEx(gamepad1);
        tuningGamepad = new ExtendedGamepadEx(gamepad2);

        bot = new Bot(telem, hardwareMap, gamepad1);

        //region Drivetrain

        //endregion

        //region Intake

        intake = bot.getIntake();

        Button intakeArmUp = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_UP))
                .whenPressed(
                        new InstantCommand(() -> intake.setIntakeArmPosition(0.85))
                );

        Button intakeArmDown = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_DOWN))
                .whenPressed(
                        new InstantCommand(() -> intake.setIntakeArmPosition(0.0))
                );

        //endregion

        //region Clipper

        clipper = bot.getClipper();

        Button clipPosition = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.CROSS))
                .whenPressed(
                        //new SetClipperPositionCommand(clipper, Clipper.ClipperState.CLIP)
                        new SetClipMagGripperPositionCommand(clipper, Clipper.ClipMagGripperState.CLOSE)
                        //new InstantCommand(() -> clipper.incrementClipperServoPosition(-0.01))
                        //new InstantCommand(() -> clipper.incrementClipMagGripperPosition(-0.01))
                );

        Button setPosition = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.CIRCLE))
                .whenPressed(
                        //new SetClipperPositionCommand(clipper, Clipper.ClipperState.SET)
                        new SetClipMagGripperPositionCommand(clipper, Clipper.ClipMagGripperState.OPEN)
                        //new InstantCommand(() -> clipper.incrementClipperServoPosition(0.01))
                        //new InstantCommand(() -> clipper.incrementClipMagGripperPosition(0.01))
                );

        Button upPosition = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.TRIANGLE))
                .whenPressed(
                        new SetClipMagPivotPositionCommand(clipper, Clipper.ClipMagState.UP)
                );

        Button downPosition = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.SQUARE))
                .whenPressed(
                        new SetClipMagPivotPositionCommand(clipper, Clipper.ClipMagState.DOWN)
                );

        //Button testPIDRight = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_UP))
        //.whenPressed(
        //new SetClipperDrivePositionCommand(clipper, clipper.magPosition)
        //);

        //Button testPIDLeft = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_DOWN))
        //.whenPressed(
        //new SetClipperDrivePositionCommand(clipper, Configuration.clipDriveClipPosition)
        //);

        Button testClip = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_RIGHT));


        ManualClipperDriveCommand manualClipperDriveCommand = new ManualClipperDriveCommand(
                clipper,
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
        );

        CommandScheduler.getInstance().registerSubsystem(clipper);
        clipper.setDefaultCommand(manualClipperDriveCommand);

        //endregion

        //region Deposit

        //endregion

        CommandScheduler.getInstance().schedule(
                //new InstantCommand(() -> {
                //clipper.setClipperServoPosition(Configuration.clipperSetPosition);
                //clipper.setClipmagPivotServoPosition(Configuration.clipMagUp);
                //})
                //new SetClipperPositionCommand(clipper, Clipper.ClipperState.CLIP)
                new SetClipMagGripperPositionCommand(clipper, Clipper.ClipMagGripperState.CLOSE)
        );

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            testClip.whenPressed(new TestClipCommand(bot));
        }

        CommandScheduler.getInstance().reset();
    }
}
