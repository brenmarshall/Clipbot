package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Configuration;
import org.firstinspires.ftc.teamcode.common.commandbase.command.clipper.SetClipMagGripperPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.clipper.TestClipCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.ManualTurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.SetIntakeArmAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.SetIntakeTurretAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.homing.HomeIntakeTurretCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.SetIntakeClawPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.SetIntakeWristAngleCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.util.ExtendedGamepadButton;
import org.firstinspires.ftc.teamcode.common.util.ExtendedGamepadEx;
import org.firstinspires.ftc.teamcode.common.util.ExtendedGamepadKeys;
import org.firstinspires.ftc.teamcode.common.util.SusParallelCommandGroup;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test", group = "TeleOp")
public class Test extends LinearOpMode {

    private Bot bot;
    private Intake intake;
    private Clipper clipper;
    private Deposit deposit;

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
                        new InstantCommand(() -> intake.setIntakeArmAngle(115.0))
                );

        Button intakeArmDown = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_DOWN))
                .whenPressed(
                        new InstantCommand(() -> intake.setIntakeArmAngle(0.0))
                );

        Button intakeWristUp = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_LEFT))
                .whenPressed(
                        new SetIntakeWristAngleCommand(intake, 180.0)
                );

        Button intakeWristDown = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_RIGHT))
                .whenPressed(
                        new SetIntakeWristAngleCommand(intake, 0.0)
                );

        Button intakeClawOpen = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.CIRCLE))
                .whenPressed(
                        new SetIntakeClawPositionCommand(intake, Intake.IntakeClawState.OPEN)
                );

        Button intakeClawClose = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.CROSS))
                .whenPressed(
                        new SetIntakeClawPositionCommand(intake, Intake.IntakeClawState.CLOSED)
                );

        Button homeIntakeTurret = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.SQUARE))
                .whenPressed(
                        new HomeIntakeTurretCommand(intake)
                );

        Button testTransfer = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.TRIANGLE))
                .whenPressed(
                        new SequentialCommandGroup(
                                new HomeIntakeTurretCommand(intake),
                                new SusParallelCommandGroup(
                                        new SetIntakeTurretAngleCommand(intake, Configuration.intakeTurretTransferAngle),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(() -> intake.getTurretAngle() >= 240.0),
                                                new SetIntakeWristAngleCommand(intake, (450 - Configuration.intakeTurretTransferAngle + 10))
                                        )
                                ),
                                new WaitCommand(250),
                                new SetIntakeArmAngleCommand(intake, 60.0),
                                new WaitCommand(1000),
                                new SetIntakeClawPositionCommand(intake, Intake.IntakeClawState.OPEN),
                                new WaitCommand(250),
                                new SetIntakeArmAngleCommand(intake, 115.0)
                        )
                );

        Button intakeTurretLeft = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.LEFT_BUMPER))
                .whenPressed(
                        new SetIntakeTurretAngleCommand(intake, 0.0)
                );

        Button intakeTurretRight = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.RIGHT_BUMPER))
                .whenPressed(
                        new SetIntakeTurretAngleCommand(intake, 180.0)
                );

        ManualTurretCommand manualTurretCommand = new ManualTurretCommand(
                intake,
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
        );

        //endregion

        //region Clipper

        clipper = bot.getClipper();

        //Button clipPosition = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.CROSS))
                //.whenPressed(
                        //new SetClipMagGripperPositionCommand(clipper, Clipper.ClipMagGripperState.CLOSE)
                //);

        //Button setPosition = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.CIRCLE))
                //.whenPressed(
                        //new SetClipMagGripperPositionCommand(clipper, Clipper.ClipMagGripperState.OPEN)
                //);

        //Button upPosition = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.TRIANGLE))
                //.whenPressed(
                        //new SetClipMagPivotPositionCommand(clipper, Clipper.ClipMagState.UP)
                //);

        //Button downPosition = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.SQUARE))
                //.whenPressed(
                        //new SetClipMagPivotPositionCommand(clipper, Clipper.ClipMagState.DOWN)
                //);

        //Button testPIDRight = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_UP))
        //.whenPressed(
        //new SetClipperDrivePositionCommand(clipper, clipper.magPosition)
        //);

        //Button testPIDLeft = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_DOWN))
        //.whenPressed(
        //new SetClipperDrivePositionCommand(clipper, Configuration.clipDriveClipPosition)
        //);

        Button testClip = (new ExtendedGamepadButton(driverGamepad, ExtendedGamepadKeys.Button.DPAD_RIGHT));


        //ManualClipperDriveCommand manualClipperDriveCommand = new ManualClipperDriveCommand(
                //clipper,
                //() -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                //() -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
        //);

        CommandScheduler.getInstance().registerSubsystem(clipper);
        //clipper.setDefaultCommand(manualClipperDriveCommand);

        //endregion

        //region Deposit

        //endregion

        CommandScheduler.getInstance().schedule(
                //new InstantCommand(() -> {
                //clipper.setClipperServoPosition(Configuration.clipperSetPosition);
                //clipper.setClipmagPivotServoPosition(Configuration.clipMagUp);
                //})
                //new SetClipperPositionCommand(clipper, Clipper.ClipperState.CLIP)
                new SetClipMagGripperPositionCommand(clipper, Clipper.ClipMagGripperState.CLOSED),
                new SetIntakeArmAngleCommand(intake, 115.0),
                new SetIntakeWristAngleCommand(intake, 90.0),
                new SetIntakeClawPositionCommand(intake, Intake.IntakeClawState.CLOSED)
                //new HomeIntakeTurretCommand(intake)
        );

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            testClip.whenPressed(new TestClipCommand(bot));

            manualTurretCommand.execute();
        }

        CommandScheduler.getInstance().reset();
    }
}
