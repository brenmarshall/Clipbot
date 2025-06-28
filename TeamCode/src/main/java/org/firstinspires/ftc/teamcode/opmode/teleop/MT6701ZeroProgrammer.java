package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // Or Autonomous if preferred for programming
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType; // Required for annotation

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

//@TeleOp(name = "MT6701 Zero Programmer", group = "Sensor Testing")
// This annotation is helpful for the FTC SDK to recognize your I2C device type.
// If you create a custom device type in your hardware configuration, you might
// need to adjust this, or remove it and rely solely on I2cDeviceSynchImpl.
//@I2cDeviceType()
public class MT6701ZeroProgrammer extends LinearOpMode {

    // --- MT6701 I2C Constants (CONFIRM THESE WITH YOUR SENSOR'S DATASHEET!) ---
    // The 7-bit I2C slave address of the MT6701 sensor.
    // THIS IS A PLACEHOLDER. Replace with the actual address of your sensor.
    // Common addresses are 0x36 or 0x0C.
    private static final byte I2C_ADDRESS = (byte) 0x36; // Example: Assuming 7-bit address 0x36

    // Register addresses for EEPROM programming
    private static final byte ZERO_MSB_REG_ADDR = (byte) 0x32; // Contains bits [11:8] of zero position
    private static final byte ZERO_LSB_REG_ADDR = (byte) 0x33; // Contains bits [7:0] of zero position
    private static final byte PROGRAM_KEY_REG_ADDR = (byte) 0x09; // Register for programming key
    private static final byte PROGRAM_COMMAND_REG_ADDR = (byte) 0x0A; // Register for programming command

    // Values to write for programming
    private static final byte PROGRAMMING_KEY_VALUE = (byte) 0xB3;
    private static final byte PROGRAMMING_COMMAND_VALUE = (byte) 0x05;

    // I2C communication object
    private I2cDeviceSynch i2cSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing MT6701 Zero Programmer...");
        telemetry.update();

        // Initialize I2C device
        // The string "mt6701_sensor" must match the name you configure in the
        // REV Control Hub's hardware configuration.
        i2cSensor = hardwareMap.get(I2cDeviceSynch.class, "mt6701_sensor");

        // Set the I2C device address
        i2cSensor.setI2cAddress(I2cAddr.create7bit(I2C_ADDRESS));

        // Activate the I2C device
        i2cSensor.engage();

        telemetry.addData("Status", "Hardware Initialized. Waiting for Start...");
        telemetry.update();

        // Optional: Read current zero position at startup for initial verification
        readCurrentZeroPosition();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "OpMode Started.");
            telemetry.update();

            // --- USER INTERACTION FOR DESIRED ZERO POSITION ---
            // For a competition robot, you might use gamepad input or a pre-defined value.
            // For this example, we'll hardcode a value or use gamepad input.

            // Example 1: Hardcode a desired zero position
            int desiredZeroPosition = 2048; // A common midpoint for 0-4095 range
            telemetry.addData("Desired Zero Position (Hardcoded)", desiredZeroPosition);
            telemetry.update();
            sleep(1000); // Give user time to read

            // Example 2: Use gamepad input for desired zero position (more advanced, requires loop)
            // This is just a conceptual example. A proper input system would be more robust.
            /*
            telemetry.addData("Instructions", "Use D-Pad Up/Down to set zero position, A to confirm");
            int gamepadZeroPosition = 0;
            while(opModeIsActive() && !gamepad1.a) {
                if (gamepad1.dpad_up) {
                    gamepadZeroPosition = Math.min(4095, gamepadZeroPosition + 10);
                } else if (gamepad1.dpad_down) {
                    gamepadZeroPosition = Math.max(0, gamepadZeroPosition - 10);
                }
                telemetry.addData("Current Selection", gamepadZeroPosition);
                telemetry.update();
                sleep(100); // Debounce
            }
            desiredZeroPosition = gamepadZeroPosition;
            telemetry.addData("Selected Zero Position", desiredZeroPosition);
            telemetry.update();
            sleep(1000);
            */

            // --- PROGRAM THE ZERO POSITION ---
            programZeroPosition(desiredZeroPosition);

            // --- VERIFY THE PROGRAMMING ---
            readCurrentZeroPosition();

            telemetry.addData("Status", "Programming attempt complete. Check telemetry for results.");
            telemetry.addData("Remember", "A power cycle of the sensor might be needed for the new zero position to take full effect after programming.");
            telemetry.update();

            while (opModeIsActive()) {
                // Keep OpMode active to view telemetry
                idle();
            }
        }

        // Disengage the I2C device when the OpMode stops
        i2cSensor.disengage();
        telemetry.addData("Status", "OpMode Ended. I2C Disengaged.");
        telemetry.update();
    }

    /**
     * Programs the desired zero position into the MT6701's EEPROM.
     * @param zeroPosition The 12-bit value (0-4095) for the new zero position.
     */
    private void programZeroPosition(int zeroPosition) {
        if (zeroPosition < 0 || zeroPosition > 4095) {
            telemetry.addData("ERROR", "Invalid zero position value: " + zeroPosition + ". Must be 0-4095.");
            telemetry.update();
            return;
        }

        telemetry.addData("Programming", "Attempting to program " + zeroPosition + "...");
        telemetry.update();

        try {
            // Step 1: Prepare the 12-bit value for MSB and LSB registers
            // ZERO_MSB_REG (0x32) contains bits [11:8] in its lower 4 bits (0-3).
            // So, value >> 8 & 0x0F isolates bits 11-8.
            byte zeroMsbData = (byte) ((zeroPosition >> 8) & 0x0F);
            byte zeroLsbData = (byte) (zeroPosition & 0xFF); // Bits [7:0]

            // Write Zero_MSB (register 0x32) and Zero_LSB (register 0x33)
            // The MT6701 datasheet specifies Address 0x32[3:0] & 0x33[7:0]
            // This means register 0x32 holds the MSB (bits 11-8) and register 0x33 holds the LSB (bits 7-0).
            // Ensure you write to the correct byte within the register.
            // For 0x32, we only care about bits 0-3. If other bits are reserved, you must read, modify, then write.
            // For simplicity, this assumes writing only the necessary bits.
            // A more robust implementation would read the register, mask, OR, then write.

            // First write the LSB
            i2cSensor.write(ZERO_LSB_REG_ADDR, new byte[]{zeroLsbData});
            telemetry.addData("Write", "LSB (0x" + String.format("%02X", ZERO_LSB_REG_ADDR) + ") = 0x" + String.format("%02X", zeroLsbData));
            telemetry.update();
            sleep(50); // Small delay between I2C writes

            // Then write the MSB
            i2cSensor.write(ZERO_MSB_REG_ADDR, new byte[]{zeroMsbData});
            telemetry.addData("Write", "MSB (0x" + String.format("%02X", ZERO_MSB_REG_ADDR) + ") = 0x" + String.format("%02X", zeroMsbData));
            telemetry.update();
            sleep(50); // Small delay

            // Step 2: Write Programming Key (0xB3) to Register 0x09
            i2cSensor.write(PROGRAM_KEY_REG_ADDR, new byte[]{PROGRAMMING_KEY_VALUE});
            telemetry.addData("Write", "Prog Key (0x" + String.format("%02X", PROGRAM_KEY_REG_ADDR) + ") = 0x" + String.format("%02X", PROGRAMMING_KEY_VALUE));
            telemetry.update();
            sleep(50); // Small delay

            // Step 3: Write Programming Command (0x05) to Register 0x0A
            i2cSensor.write(PROGRAM_COMMAND_REG_ADDR, new byte[]{PROGRAMMING_COMMAND_VALUE});
            telemetry.addData("Write", "Prog Cmd (0x" + String.format("%02X", PROGRAM_COMMAND_REG_ADDR) + ") = 0x" + String.format("%02X", PROGRAMMING_COMMAND_VALUE));
            telemetry.update();

            // Step 4: Wait for more than 600ms (as per datasheet)
            telemetry.addData("Programming", "Waiting 600ms for EEPROM write cycle...");
            telemetry.update();
            sleep(600); // Crucial delay for EEPROM programming

            telemetry.addData("Programming", "Programming sequence sent. Verify by reading after power cycle.");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("ERROR", "I2C Programming Failed: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Reads the current zero position from the MT6701's EEPROM.
     */
    private void readCurrentZeroPosition() {
        telemetry.addData("Reading", "Attempting to read current zero position...");
        telemetry.update();

        try {
            // Read LSB (8 bits)
            byte[] lsbBytes = i2cSensor.read(ZERO_LSB_REG_ADDR, 1);
            byte lsb = (lsbBytes != null && lsbBytes.length > 0) ? lsbBytes[0] : 0;

            // Read MSB (4 bits)
            byte[] msbBytes = i2cSensor.read(ZERO_MSB_REG_ADDR, 1);
            // Only the lower 4 bits of the MSB register are used for the 12-bit value.
            // So, mask out any higher bits if they happen to be set.
            byte msb = (msbBytes != null && msbBytes.length > 0) ? (byte)(msbBytes[0] & 0x0F) : 0;

            // Reconstruct the 12-bit value
            int currentZeroPosition = ((msb & 0xFF) << 8) | (lsb & 0xFF); // Use & 0xFF to handle signed bytes

            telemetry.addData("Read Zero Position", currentZeroPosition);
            telemetry.addData("Raw MSB (0x" + String.format("%02X", ZERO_MSB_REG_ADDR) + ")", "0x" + String.format("%02X", msb));
            telemetry.addData("Raw LSB (0x" + String.format("%02X", ZERO_LSB_REG_ADDR) + ")", "0x" + String.format("%02X", lsb));
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("ERROR", "I2C Read Failed: " + e.getMessage());
            telemetry.update();
        }
    }
}
