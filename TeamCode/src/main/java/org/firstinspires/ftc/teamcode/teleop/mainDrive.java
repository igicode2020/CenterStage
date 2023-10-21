package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name="MainDrive", group="--")
// @Disabled
public class mainDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FRM = null;
    private DcMotor BRM = null;
    private DcMotor FLM = null;
    private DcMotor BLM = null;

    double FRPower, BRPower, FLPower, BLPower;

    //set constants
    double directionMultiplier = 0.5;
    double intakePower = 1;
    double outtakePower = 1;

    // default value
    double slide_encoder_value = 0;

    // Setting up Slug Mode Parameters
    boolean slugMode = false;
    double slugMultiplier = 0.33;

    // set up imu for gyro
    // BNO055IMU is the orientation sensor
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // get motors, linear slide, and wheel intakes from hardware map
        FRM = hardwareMap.get(DcMotorEx.class, "frontRight");
        BRM = hardwareMap.get(DcMotorEx.class, "backRight");
        FLM = hardwareMap.get(DcMotorEx.class, "frontLeft");
        BLM = hardwareMap.get(DcMotorEx.class, "backLeft");

        //GamePads to save previous state of gamepad for button toggling
        Gamepad previousGamePad1 = new Gamepad();
        Gamepad currentGamePad1 = new Gamepad();
        Gamepad previousGamePad2 = new Gamepad();
        Gamepad currentGamePad2 = new Gamepad();

        //setting motor parameters
        FRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        FRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        BRM.setDirection(DcMotorEx.Direction.REVERSE);
        FRM.setDirection(DcMotorEx.Direction.REVERSE);

        // Setting parameters for imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //direction for headless mode
        double direction;
        double speed;

        boolean headlessMode = true;

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            previousGamePad1.copy(currentGamePad1);
            currentGamePad1.copy(gamepad1);
            previousGamePad2.copy(currentGamePad2);
            currentGamePad2.copy(gamepad2);

            /*try{
                //setting previous state of gamepad1 and current position for later use toggling slug mode
                previousGamePad1.copy(currentGamePad1);
                currentGamePad1.copy(gamepad1);
                previousGamePad2.copy(currentGamePad2);
                currentGamePad2.copy(gamepad2);
            }
                catch(RobotCoreException e){
            //}*/

            if(currentGamePad1.right_bumper && !previousGamePad1.right_bumper){
                headlessMode = !headlessMode;
            }

            if(headlessMode) {
                telemetry.addData("Right trigger", currentGamePad2.right_trigger);
                telemetry.addData("Left trigger", currentGamePad2.left_trigger);

                if (currentGamePad1.start && !previousGamePad1.start) {
                    parameters = new BNO055IMU.Parameters();
                    parameters.mode = BNO055IMU.SensorMode.IMU;
                    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                    parameters.loggingEnabled = false;

                    imu.initialize(parameters);
                }

                // button a to toggle slug mode
                if (currentGamePad1.left_bumper && !previousGamePad1.left_bumper) {
                    slugMode = !slugMode;
                }

                // setting direction, atan2 gives theta in polar coordinates
                direction = Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y) - (getAngle() * (Math.PI / 180) - (Math.PI / 2));
                speed = Math.min(1.0, Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y));

//          Math.min is to make sure the powers aren't set to anything more than one for now
//          Dividing by 28 to get a number between -1 and 1 (power percentage instead of ticks/sec)
//          Multiply by -1 to reverse the direction of the robot
                FLPower = (speed * Math.sin(direction + Math.PI / 4.0) - directionMultiplier * gamepad1.right_stick_x);
                FRPower = -(speed * Math.cos(direction + Math.PI / 4.0) - directionMultiplier * gamepad1.right_stick_x);
                BLPower = -(speed * Math.cos(direction + Math.PI / 4.0) + directionMultiplier * gamepad1.right_stick_x);
                BRPower = (speed * Math.sin(direction + Math.PI / 4.0) + directionMultiplier * gamepad1.right_stick_x);

                if (slugMode) {
                    FRPower = FRPower * slugMultiplier;
                    BRPower = BRPower * slugMultiplier;
                    FLPower = FLPower * slugMultiplier;
                    BLPower = BLPower * slugMultiplier;
                }

                FRM.setPower(FRPower);
                BRM.setPower(BRPower);
                FLM.setPower(FLPower);
                BLM.setPower(BLPower);
            }
            else{

                // button a to toggle slug mode
                if (currentGamePad1.left_bumper && !previousGamePad1.left_bumper) {
                    slugMode = !slugMode;
                }

                FRPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - directionMultiplier*gamepad1.right_stick_x);
                BRPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - directionMultiplier*gamepad1.right_stick_x);
                FLPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + directionMultiplier*gamepad1.right_stick_x);
                BLPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + directionMultiplier*gamepad1.right_stick_x);

                if (slugMode) {
                    FRPower = FRPower * slugMultiplier;
                    BRPower = BRPower * slugMultiplier;
                    FLPower = FLPower * slugMultiplier;
                    BLPower = BLPower * slugMultiplier;
                }

                FRM.setPower(-FRPower);
                BRM.setPower(-BRPower);
                FLM.setPower(-FLPower);
                BLM.setPower(-BLPower);
            }

            telemetry.addData("slide position", slide_encoder_value);
            telemetry.addData("FRPower", FRPower);
            telemetry.addData("BRPower", BRPower);
            telemetry.addData("FLPower", FLPower);
            telemetry.addData("BLPower", BLPower);
            telemetry.addData("Direction", getAngle());
            telemetry.addData("Accelerometer", imu.getAcceleration());
            telemetry.update();

        }
    }

    private double getAngle() {
        /* We experimentally determined the Z axis is the axis we want to use for heading angle.
           We have to process the angle because the imu works in euler angles so the Z axis is
           returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
           180 degrees. We detect this transition and track the total cumulative angle of rotation. */

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        // return globalAngle;
        return globalAngle;
    }
}