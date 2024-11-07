package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.ToggleButton;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.USBCamera;

import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;


@TeleOp(name="Mechanum", group="Iterative Opmode")
public class Robot extends OpMode {
    
    private DcMotor armPivot;
    private Servo clawLeft;
    private Servo clawRight;
    private Servo wristPivot;
    
    private Servo drone;
    
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private IMU imu;
    private YawPitchRollAngles gyro;
    private OpenCvWebcam webcam;

    private ToggleButton toggle_leftBumper;
    private ToggleButton toggle_rightBumper;
    private ToggleButton toggle_rightTrigger;
    private ToggleButton toggle_a;

    private Arm arm;
    private Claw claw;
    private Drivetrain mecanum;
    private Wrist wrist;
    // private USBCamera camera;
    
    private String currentMode = "idle";
    private String placeMode = "frontLow";
    


    @Override
    public void init() {
        armPivot = hardwareMap.get(DcMotor.class, "armPivot");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        wristPivot = hardwareMap.get(Servo.class, "wristPivot");
        
        drone = hardwareMap.get(Servo.class, "drone");
        
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        
        imu = createIMU();
        // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        imu.resetYaw();
        
        // Toggle Objects
        toggle_leftBumper = new ToggleButton();
        toggle_rightBumper = new ToggleButton();
        toggle_rightTrigger = new ToggleButton();
        toggle_a = new ToggleButton();

        // Subsystems
        arm = new Arm(armPivot);
        claw = new Claw(clawLeft, clawRight);
        mecanum = new Drivetrain(frontLeft, frontRight, backLeft, backRight);
        wrist = new Wrist(wristPivot);
        // camera = new USBCamera(webcam);
    }



    public void updateData() {
        telemetry.update();
    }
    
    public IMU createIMU() {
        return hardwareMap.get(IMU.class, "imu");
    }
    


    @Override
    public void loop() {
        // IMU
        gyro = imu.getRobotYawPitchRollAngles();

        // Sticks
        double translateX = gamepad1.left_stick_x;
        double translateY = gamepad1.left_stick_y * -1;
        double rotate = gamepad1.right_stick_x * 0.5;

        // Buttons and Triggers
        boolean leftBumper = gamepad1.left_bumper; // Toggle Left Claw
        boolean rightBumper = gamepad1.right_bumper; // Toggle Right Claw
        boolean leftTrigger = gamepad1.left_trigger > 0.5; // Hold to grab
        boolean rightTrigger = gamepad1.right_trigger > 0.5; // Toggle to place

        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        
        // Telemetry
        telemetry.addData("Arm Position", String.valueOf(armPivot.getCurrentPosition()));
        telemetry.addData("Left Claw", String.valueOf(clawLeft.getPosition()));
        telemetry.addData("Right Claw", String.valueOf(clawRight.getPosition()));
        telemetry.addData("Wrist", String.valueOf(wristPivot.getPosition()));
        telemetry.addData("Current Mode", String.valueOf(currentMode));
        telemetry.addData("IMU state", String.valueOf(gyro.getYaw(AngleUnit.DEGREES)));
        // telemetry.addData("Detected X", String.valueOf(camera.getBestDetectedPosition()[0]));
        // telemetry.addData("Detected Y", String.valueOf(camera.getBestDetectedPosition()[1]));


        // Loops
        updateData();
        mecanum.runMecanum(translateX, translateY, rotate, gyro.getYaw(AngleUnit.RADIANS));

        func_grab(leftTrigger);
        func_clawLeft(toggle_leftBumper, leftBumper);
        func_clawRight(toggle_rightBumper, rightBumper);
        func_armPlace(toggle_rightTrigger, rightTrigger, toggle_rightBumper, toggle_leftBumper);
        func_changePlaceMode(toggle_a, a);
        func_resetIMU(b);
        func_drone(x);

        updateAll();
    }



    public void updateAll() {
        arm.update();
        claw.update();
        wrist.update();
    }


    public boolean previousTriggerState = false;
    public void func_grab(boolean leftTrigger) {
        if ((currentMode == "idle" || currentMode == "grab") && currentMode != "place") {

            
            if (leftTrigger) {
                currentMode = "grab";
                claw.clawOpen();
                wrist.wristPickup();
            } else if (currentMode == "grab") {
                new Thread(() -> {
                    try {
                        claw.clawClamp();
                        claw.update();
                        Thread.sleep(300);
                        wrist.wristStore();
                        currentMode = "idle";
                    } catch (Exception e) {}
                }).start();
            }
        }
    }
    
    

    public void func_clawLeft(ToggleButton inputObj, boolean input) {
        if (currentMode == "place") {

            if (inputObj.toggle(input)) {
                claw.clawOpenLeft();
            } else {
                claw.clawClampLeft();
            }
        }
    }



    public void func_clawRight(ToggleButton inputObj, boolean input) {
        if (currentMode == "place") {

            if (inputObj.toggle(input)) {
                claw.clawOpenRight();
            } else {
                claw.clawClampRight();
            }
        }
    }

    public void func_armPlace(ToggleButton inputObj, boolean input, ToggleButton bumperLeft, ToggleButton bumperRight) {
        if ((currentMode == "idle" || currentMode == "place") && currentMode != "grab") {

            if (inputObj.toggle(input)) {
                arm.armPlace(placeMode);
                wrist.wristPlace(placeMode);
                currentMode = "place";
            } else {
                arm.armStore();
                wrist.wristStore();
                claw.clawClampLeft();
                claw.clawClampRight();
                bumperLeft.forceState(false);
                bumperRight.forceState(false);
                currentMode = "idle";
            }
        }
    }
    
    public void func_changePlaceMode(ToggleButton inputObj, boolean input) {
        if (inputObj.toggle(input)) {
            placeMode = "backHigh";
        } else {
            placeMode = "frontLow";
        }
    }
    
    public void func_resetIMU(boolean input) {
        if (input) {
            new Thread(() -> {
                try {
                    imu.close();
                    Thread.sleep(100);
                    imu = createIMU();
                    Thread.sleep(100);
                    imu.resetYaw();
                    Thread.sleep(100);
                } catch (Exception e) {}
            }).start();
        }
    }
    
    public void func_drone(boolean input) {
        if (input) {
            drone.setPosition(1);
        }
    }
}