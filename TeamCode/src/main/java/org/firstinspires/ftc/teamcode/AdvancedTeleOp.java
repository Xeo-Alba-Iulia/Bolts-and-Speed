package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Advanced TeleOp", group="TeleOp")
public class AdvancedTeleOp extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor turnMotor;
    private Servo clawMotor;

    @Override
    public void init() {
        // Initialize hardware
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        turnMotor = hardwareMap.get(DcMotor.class, "turn_drive");
        clawMotor = hardwareMap.get(Servo.class, "claw_drive");

        turnMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        double joy_axis = gamepad1.left_stick_x;
        boolean claw = gamepad1.circle;
        boolean hand_break = gamepad1.square;

        // Call the movement logic function
        MovementOutput output = movementLogic(lt, rt, joy_axis);

        if (!output.isValid) {
            telemetry.addData("Error", "Invalid inputs");
        } else if (!hand_break){
            // Set motor power
            leftMotor.setPower(output.leftMotor);
            rightMotor.setPower(output.rightMotor);
            int currentPos = turnMotor.getCurrentPosition();
            if (Math.abs(currentPos - output.turnAngle) > 5) { // Adjust the threshold as needed
                turnMotor.setPower(currentPos < output.turnAngle ? 1.0 : -1.0);
            } else {
                turnMotor.setPower(0);
            }


            // Telemetry data for debugging
            telemetry.addData("Left Motor Power", output.leftMotor);
            telemetry.addData("Right Motor Power", output.rightMotor);
            telemetry.addData("Turn Motor Position", output.turnAngle);
            telemetry.addData("Real turn angle position:", turnMotor.getCurrentPosition());
            telemetry.addData("Servo position", clawMotor.getPosition());
        }
        else {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setTargetPosition(rightMotor.getCurrentPosition());
            rightMotor.setPower(-1);
            leftMotor.setPower(0);
        }
        telemetry.update();
        if (claw) {
            clawMotor.setPosition(0.0);
        }
        else {
            clawMotor.setPosition(1.0);
        }
    }

    private MovementOutput movementLogic(double lt, double rt, double joy_axis) {
        MovementOutput output = new MovementOutput();

        double turn_angle = 70;
        double max_voltage = 1.0;
        double diff_str = 3;

        if (!(lt >= -1.1 && lt <= 1.1 && rt >= -1.1 && rt <= 1.1 && joy_axis >= -1.1 && joy_axis <= 1.1)) {
            output.isValid = false;
            return output;
        }

        output.isValid = true;
        double speed = rt - lt;
        output.turnAngle = (int) (joy_axis * Math.abs(joy_axis) * -turn_angle);
        double l_motor =  speed * max_voltage;
        double r_motor =  speed * max_voltage;

        output.leftMotor = Math.max(-max_voltage, Math.min(l_motor, max_voltage));
        output.rightMotor = Math.max(-max_voltage, Math.min(r_motor, max_voltage));

        return output;
    }

    private static class MovementOutput {
        boolean isValid;
        int turnAngle;
        double leftMotor;
        double rightMotor;
    }
}
