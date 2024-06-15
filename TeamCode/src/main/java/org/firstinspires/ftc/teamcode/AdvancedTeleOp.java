package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Advanced TeleOp", group="TeleOp")
public class AdvancedTeleOp extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor turnMotor;

    @Override
    public void init() {
        // Initialize hardware
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        turnMotor = hardwareMap.get(DcMotor.class, "turn_drive");
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
            turnMotor.setTargetPosition(output.turnAngle);

            // Telemetry data for debugging
            telemetry.addData("Left Motor Power", output.leftMotor);
            telemetry.addData("Right Motor Power", output.rightMotor);
            telemetry.addData("Turn Motor Position", output.turnAngle);
        }
        telemetry.update();
    }

    private MovementOutput movementLogic(double lt, double rt, double joy_axis) {
        MovementOutput output = new MovementOutput();

        double turn_angle = 30;
        double max_voltage = 1;
        double diff_str = 3;

        if (!(lt >= -1.1 && lt <= 1.1 && rt >= -1.1 && rt <= 1.1 && joy_axis >= -1.1 && joy_axis <= 1.1)) {
            output.isValid = false;
            return output;
        }

        output.isValid = true;
        double speed = rt - lt;
        output.turnAngle = (int) (joy_axis * Math.abs(joy_axis) * turn_angle);
        double l_motor = (-speed - joy_axis / diff_str) * max_voltage;
        double r_motor = (speed + joy_axis / diff_str) * max_voltage;

        output.leftMotor = Math.min(l_motor, max_voltage);
        output.rightMotor = Math.min(r_motor, max_voltage);

        return output;
    }

    private static class MovementOutput {
        boolean isValid;
        int turnAngle;
        double leftMotor;
        double rightMotor;
    }
}
