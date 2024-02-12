package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="config", group="--")
public class MotorCurrentDetection extends LinearOpMode {

    private DcMotor slide;
    private final double MAX_CURRENT_THRESHOLD = 10.0; // Adjust this value as needed
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        slide = hardwareMap.get(DcMotor.class, "linearMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (currentGamePad1.right_trigger > 0) {
                slide.setPower(currentGamePad1.right_trigger);
            }
            double current = slide.getCurrent(CurrentUnit.AMPS);
            if (current >= MAX_CURRENT_THRESHOLD) {
                // Motor current is reaching the max threshold
                // Set the current position as the max height
                slide.setMaxHeight(slide.getCurrentPosition());
                
                // You may want to add some additional logic here
                // like logging the event
                slide.setPower(0);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Current", "%.2f Amps", current);
            telemetry.update();
        }
    }
}