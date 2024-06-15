package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Advanced TeleOp", group="TeleOp")
public class AdvancedTeleOp extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    @Override
    public void init() {
        // Initialize hardware
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
    }

    @Override
    public void loop() {
        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        double joy_axis = gamepad1.left_stick_x;

        // Call the movement logic function
        MovementOutput output = movementLogic(lt, rt, joy_axis);

        if (!output.isValid) {
            telemetry.addData("Error", "Invalid inputs");
        } else {
            // Set motor power
            leftMotor.setPower(output.leftMotor);
            rightMotor.setPower(output.rightMotor);

            // Telemetry data for debugging
            telemetry.addData("Left Motor Power", output.leftMotor);
            telemetry.addData("Right Motor Power", output.rightMotor);
        }
        telemetry.update();
    }

    private MovementOutput movementLogic(double lt, double rt, double joy_axis) {
        MovementOutput output = new MovementOutput();

        double turn_angle = 30;
        double max_voltage = 12;
        double diff_str = 3;

        if (!(lt >= -1.1 && lt <= 1.1 && rt >= -1.1 && rt <= 1.1 && joy_axis >= -1.1 && joy_axis <= 1.1)) {
            output.isValid = false;
            return output;
        }

        output.isValid = true;
        double speed = rt - lt;
        output.turnAngle = joy_axis * Math.abs(joy_axis) * turn_angle;
        double l_motor = -speed - joy_axis / diff_str;
        double r_motor = speed + joy_axis / diff_str;

        output.leftMotor = Math.min(l_motor, max_voltage);
        output.rightMotor = Math.min(r_motor, max_voltage);

        return output;
    }

    private static class MovementOutput {
        boolean isValid;
        double turnAngle;
        double leftMotor;
        double rightMotor;
    }
}
