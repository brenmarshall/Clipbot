package org.firstinspires.ftc.teamcode.common.commandbase.command.intake.homing;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.homing.ApproachLimitSwitch;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.homing.BackoffFromLimitSwitch;
import org.firstinspires.ftc.teamcode.common.commandbase.command.intake.homing.ZeroTurretAtLimit;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class HomeIntakeTurretCommand extends SequentialCommandGroup {

    // --- TUNING CONSTANTS ---
    // Adjust these values for your specific robot's weight, friction, and motor power.
    private static final double FAST_APPROACH_POWER = 1.0;  // Moderate speed for the first pass
    private static final double BACKOFF_POWER = 0.2;        // Gentle speed for backing off
    private static final long BACKOFF_DURATION_MS = 300;    // How long to back off (in milliseconds)
    private static final double SLOW_APPROACH_POWER = 0.1;  // Very slow speed for the final, precise touch

    /**
     * Creates a multi-stage, high-precision homing command for the intake turret.
     * This command should be scheduled to home the turret.
     */
    public HomeIntakeTurretCommand(Intake intake) {
        addCommands(
                // Stage 1: Move towards the limit switch at a moderate speed.
                new ApproachLimitSwitch(intake, FAST_APPROACH_POWER),

                // Stage 2: Once hit, back off the switch for a short, fixed time.
                new BackoffFromLimitSwitch(intake, BACKOFF_POWER, BACKOFF_DURATION_MS),

                // Stage 3: Creep towards the switch again, very slowly, for a precise touch.
                new ApproachLimitSwitch(intake, SLOW_APPROACH_POWER),

                // Stage 4: The instant the switch is triggered, stop and zero the encoders.
                new ZeroTurretAtLimit(intake)
        );
    }
}