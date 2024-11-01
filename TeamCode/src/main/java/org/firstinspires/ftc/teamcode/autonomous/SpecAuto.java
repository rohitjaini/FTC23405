package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SpecAuto extends LinearOpMode {

    // Declare motors and servos
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor rightSlideMotor;
    private Servo rightWristServo;
    private Servo specServo;
    private CRServo activeIntake;
    private DcMotor intakeArmMotor;
    private Servo bucketServo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        rightSlideMotor = hardwareMap.dcMotor.get("rightSlideMotor");
        rightWristServo = hardwareMap.servo.get("rightWristServo");
        specServo = hardwareMap.servo.get("specServo");
        activeIntake = hardwareMap.crservo.get("activeIntake");
        intakeArmMotor = hardwareMap.dcMotor.get("intakeArmMotor");
        bucketServo = hardwareMap.servo.get("bucketServo");

        // Initialize motors
        intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmMotor.setTargetPosition(0);
        intakeArmMotor.setPower(0.3);

        intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        printTelemetry("Waiting for start...");
        waitForStart();

        // Drive backwards from wall
        printTelemetry("Driving backwards from wall");
        driveMotors(-0.5, 600);

        sleep(500);

        // Arm out for slides
        printTelemetry("Arm moving out for slides");
        intakeArmMotor.setTargetPosition(300);
        sleep(1000); // Wait for arm to reach position

        waitForInput();
        // Slides up
        printTelemetry("Slides moving up");
        rightSlideMotor.setPower(-0.5);
        sleep(3000); // Wait for slides to move up

        // Drive back
        printTelemetry("Driving back");
        driveMotors(-0.5, 200);

        // Slides down and activate spec
        printTelemetry("Slides Down");
        rightSlideMotor.setPower(0.5);
        sleep(500);
        specServo.setPosition(0.8);
        sleep(100);
        specServo.setPosition(0.2);
        sleep(300);

        // Drive away from sub (forward)
        driveMotors(0.5, 300);
    }

    // Method to drive motors
    private void driveMotors(double speed, int duration) {
        DcMotor[] driveMotors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
        for (DcMotor motor : driveMotors) {
            motor.setPower(speed);
        }
        sleep(duration);
        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
        }
    }

    // Method to print telemetry
    private void printTelemetry(String message) {
        telemetry.setAutoClear(false);
        telemetry.addLine(message);
        telemetry.update();
    }

    private void waitForInput(){
        printTelemetry("Waiting for gamepad1.a");
        while (true){
            if (gamepad1.a){
                return;
            }
            sleep(100);
        }
    }
}
