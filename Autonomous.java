package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@TeleOp(name="Autonomous", group="Iterative OpMode")
public class Autonomous extends OpMode {
  
  // Initialize and construct objects
  private DcMotor frontLeft;
  private DcMotor frontRight;
  private DcMotor backLeft;
  private DcMotor backRight;

  private Arm arm;
  private Claw claw;
  private Drivetrain mecanum;
  private Wrist wrist;

  private DcMotor armPivot;
  private Servo clawLeft;
  private Servo clawRight;
  private Servo wristPivot;


  // Path for autonomous: (x value, y value, speed)
  public double[][] path = Constants.AutoConstants.currentPath;
  public String[][] actions = Constants.AutoConstants.currentActions;
  
  public int step = 0;
  public String auto = "idle"; // idle, pending, true
  public int delay = 3; // in seconds

  // Constants (Change to real constants script?)
  public final int tolerance = Constants.AutoConstants.tolerance; 
  public final double stepsPerRotation = Constants.AutoConstants.stepsPerRotation;
  
  @Override
  public void init() {
    
    // Retrieve motors
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    backRight = hardwareMap.get(DcMotor.class, "backRight");

    armPivot = hardwareMap.get(DcMotor.class, "armPivot");
    clawLeft = hardwareMap.get(Servo.class, "clawLeft");
    clawRight = hardwareMap.get(Servo.class, "clawRight");
    wristPivot = hardwareMap.get(Servo.class, "wristPivot");

    arm = new Arm(armPivot);
    claw = new Claw(clawLeft, clawRight);
    mecanum = new Drivetrain(frontLeft, frontRight, backLeft, backRight);
    wrist = new Wrist(wristPivot);
    
    claw.clawClamp();
    claw.update();
  }
  
  @Override
  public void loop() {
    updateData();
    
    if (auto == "idle") {
      
      new Thread(() -> {
        try {
          Thread.sleep(delay * 1000);
          auto = "true";
        } catch (Exception e) {}
      }).start();
      
      auto = "pending";
    }
    
    if (auto == "true") {
      checkStep();
    }
  }
  
  public void updateData() {
    telemetry.update();
  }

  public void checkStep() {
    // Start autonomous, looping until at setpoint, then move to next position in path
    if (step == 0) {
      resetEncoders();
      movePos(path[step][0], path[step][1], path[step][2]);
      // add sleep statement here for more time to do things?
    }
    if (
      atPos(frontLeft.getCurrentPosition(), frontLeft.getTargetPosition()) &&
      atPos(frontRight.getCurrentPosition(), frontRight.getTargetPosition()) &&
      atPos(backLeft.getCurrentPosition(), backLeft.getTargetPosition()) &&
      atPos(backRight.getCurrentPosition(), backRight.getTargetPosition())
    ) {
      executeConditions(step);
      resetEncoders();
      if (step < path.length) {
        movePos(path[step][0], path[step][1], path[step][2]);
      }
    }
  }

  public void executeConditions(int step) {
    telemetry.addData("step", String.valueOf(step));

    for (int i = 0; i < actions.length; i++) {
      if (((step - 1) == i) && (actions[i] != null)) {
        for (int j = 0; j < actions[i].length; j++) {
          executeAction(actions[i][j]);
          telemetry.addData("action", String.valueOf(actions[i][j]));
        }
      }
    }
    
    arm.update();
    claw.update();
    wrist.update();
  }

  public void executeAction(String action) {
    
    switch (action) {
      case "wrist_store":
        wrist.wristStore();
        break;
      case "claw_open":
        claw.clawOpen();
        break;
      case "arm_frontLow":
        arm.armPlace("frontLow");
        break;
      case "wrist_frontLow":
        wrist.wristPlace("frontLow");
        break;
      case "arm_store":
        arm.armStore();
        break;
      case "claw_clamp":
        claw.clawClamp();
        break;
      case "wrist_pickup":
        wrist.wristPickup();
        break;
    }
  }
  
  public void resetEncoders() {
    // Reset all drivetrain encoders upon initialization and then set them to use for autonomous
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  public boolean atPos(int real, int target) {
    // Check if real position is near or at target position using tolerance
    return (Math.abs(target - real) < tolerance);
  }
  
  public void movePos(double x, double y, double speed) {
    // Convert to polar coordinates, and create a normalized r and true r
    double r = Math.hypot(x / (x + y), y / (x + y));
    double rTrue = Math.hypot(x, y);
    double robotAngle = Math.atan2(y, x) - Math.PI / 4;

    // Calculate wheel speeds and then multiply for real speed, then multiply by desired speed
    double[] speeds = {
        r * Math.cos(robotAngle) * Math.sqrt(2) * speed,
        r * Math.sin(robotAngle) * Math.sqrt(2) * speed,
        r * Math.sin(robotAngle) * Math.sqrt(2) * speed,
        r * Math.cos(robotAngle) * Math.sqrt(2) * speed
    };

    // Calculate distances based off true r and then multiply by steps per rotation
    double[] distances = {
      speeds[0] * rTrue * stepsPerRotation,
      speeds[1] * rTrue * stepsPerRotation,
      speeds[2] * rTrue * stepsPerRotation,
      speeds[3] * rTrue * stepsPerRotation
    };
    
    telemetry.addData("r", String.valueOf(r));
    telemetry.addData("Speeds0", String.valueOf(speeds[0]));
    telemetry.addData("Speeds1", String.valueOf(speeds[1]));
    telemetry.addData("Speeds2", String.valueOf(speeds[2]));
    telemetry.addData("Speeds3", String.valueOf(speeds[3]));
    telemetry.addData("Distances0", String.valueOf(distances[0]));
    telemetry.addData("Distances1", String.valueOf(distances[1]));
    telemetry.addData("Distances2", String.valueOf(distances[2]));
    telemetry.addData("Distances3", String.valueOf(distances[3]));

    frontLeft.setTargetPosition((int) Math.round(-distances[0]));
    frontRight.setTargetPosition((int) Math.round(distances[1]));
    backLeft.setTargetPosition((int) Math.round(-distances[2]));
    backRight.setTargetPosition((int) Math.round(distances[3]));

    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    // Set power to drivetrain motors
    // This must use the correct speed for the motors 
    // so that it arrives at the desired position at
    // the same time as the others.
    frontLeft.setPower(-speeds[0]);
    frontRight.setPower(speeds[1]);
    backLeft.setPower(-speeds[2]);
    backRight.setPower(speeds[3]);

    step += 1;
  }
}