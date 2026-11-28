package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.vision.yolo.Yolo26Processor;
import org.firstinspires.ftc.teamcode.vision.yolo.YoloDetection;
import org.firstinspires.ftc.vision.VisionPortal;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Concept: Ultralytics YOLO26 running on the REV Control Hub.
 *
 * <p>Lists every ball YOLO26 finds in telemetry, and draws the boxes on the camera image. Watch it live
 * in FTC Dashboard (http://192.168.43.1:8080/dash on the robot's Wi-Fi), or on the Driver Station
 * (menu, Camera Stream, during INIT). It is also a speed test: switch between the models and thread
 * counts, and the "Speed results" table in telemetry keeps the average for each combination.
 *
 * <p>Setup: a Logitech C270 named "Webcam 1" in the active robot configuration. The models live in
 * TeamCode/src/main/assets/yolo/ and are produced by ftc_techit_yolo/export_for_control_hub.py.
 * Model files (with their .properties) in FIRST/yolo on the robot's storage are added to the list
 * too, so new models can be tried without rebuilding the app: /sdcard/FIRST/yolo on the Control
 * Hub (adb push), ~/.vch/sdcard/FIRST/yolo on the virtual Control Hub.
 *
 * <p>Gamepad 1:
 * <ul>
 *   <li>dpad right / left: next / previous model</li>
 *   <li>dpad up / down: CPU threads used for inference (1 to 4)</li>
 *   <li>A: pause / resume detection</li>
 * </ul>
 */
@TeleOp(name = "Concept: YOLO26 Object Detection", group = "Concept")
public class ConceptYolo26 extends LinearOpMode {

    /** The models to compare, recommended first. TeamCode/YOLO26.md lists their accuracy. */
    private static final String[] MODELS = {
            "yolo/balls26n_256x192_int8.tflite",     // recommended: YOLO26n retrained at 256 px, INT8
            "yolo/balls26n_192x160_int8.tflite",     // faster: smaller input
            "yolo/balls26half_192x160_int8.tflite",  // fastest: half-width network, misses more far balls
            "yolo/balls26n_320x256_int8.tflite",     // slowest INT8, most pixels per ball: best for far balls
            "yolo/balls26n_320.tflite",              // the first model (train-19, FP32, 320x320): about 3 FPS
            "yolo/balls26n_256x192.onnx",            // the default model's network in FP32 on OpenCV DNN: the
                                                     // fallback if LiteRT fails, and what the PC emulator runs
    };
    private static final int MAX_THREADS = 4;            // the Control Hub has 4 Cortex-A53 cores
    private static final int FRAMES_BEFORE_RESULT = 30;  // let the averages settle before recording them
    private static final int MAX_LISTED_DETECTIONS = 5;
    private static final double DASHBOARD_STREAM_FPS = 10;  // caps the Hub's work for the live view

    /** Extra models to try: /sdcard/FIRST/yolo on the Control Hub, ~/.vch/sdcard/FIRST/yolo in the emulator. */
    private static final File EXTRA_MODELS_FOLDER = new File(AppUtil.FIRST_FOLDER, "yolo");

    @Override
    public void runOpMode() {
        List<String> models = modelList();
        int modelIndex = 0;
        int threads = MAX_THREADS;
        String loadError = null;

        // Use the first model that loads.
        Yolo26Processor yolo = null;
        for (int i = 0; i < models.size() && yolo == null; i++) {
            try {
                yolo = new Yolo26Processor.Builder()
                        .setModel(models.get(i))
                        .setNumThreads(threads)
                        .setMinConfidence(0.40f)
                        .build();
                modelIndex = i;
            } catch (RuntimeException e) {
                loadError = e.getMessage();
                RobotLog.ee("ConceptYolo26", e, "Could not load %s", models.get(i));
            }
        }
        if (yolo == null) {
            telemetry.addLine("No YOLO26 model could be loaded:");
            telemetry.addLine(loadError);
            telemetry.update();
            waitForStart();
            return;
        }

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(yolo)
                .build();

        // Live camera image with the YOLO boxes in FTC Dashboard: http://192.168.43.1:8080/dash on a
        // laptop or phone joined to the robot's Wi-Fi. Frames are only sent while the page is open.
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) dashboard.startCameraStream(portal, DASHBOARD_STREAM_FPS);

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        boolean paused = false;
        Map<String, String> speedResults = new LinkedHashMap<>();  // "model threads" -> "ms fps"

        while (opModeInInit() || opModeIsActive()) {
            // Read every "was pressed" once per loop: reading it clears it.
            boolean nextModel = gamepad1.dpadRightWasPressed();
            boolean previousModel = gamepad1.dpadLeftWasPressed();
            boolean moreThreads = gamepad1.dpadUpWasPressed();
            boolean fewerThreads = gamepad1.dpadDownWasPressed();
            boolean togglePause = gamepad1.aWasPressed();

            // Keep the latest averages for this model and thread count; they restart after a switch.
            if (!paused && yolo.getFrameCount() >= FRAMES_BEFORE_RESULT) {
                speedResults.put(String.format(Locale.US, "%-24s %dt", modelName(yolo.getModelPath()), threads),
                        String.format(Locale.US, "%4.0f ms %5.1f fps",
                                yolo.getAverageInferenceMs(), yolo.getProcessingFps()));
            }

            if (nextModel || previousModel || moreThreads || fewerThreads) {
                int step = (nextModel ? 1 : 0) - (previousModel ? 1 : 0);
                int newIndex = (modelIndex + step + models.size()) % models.size();
                int newThreads = Range.clip(threads + (moreThreads ? 1 : 0) - (fewerThreads ? 1 : 0), 1, MAX_THREADS);
                telemetry.addLine("Loading " + models.get(newIndex) + " with " + newThreads + " threads...");
                telemetry.update();
                try {
                    yolo.setModel(models.get(newIndex), newThreads);
                    modelIndex = newIndex;
                    threads = newThreads;
                    loadError = null;
                } catch (IOException e) {
                    loadError = e.getMessage();  // the previous model keeps running
                }
            }
            if (togglePause) {
                paused = !paused;
                portal.setProcessorEnabled(yolo, !paused);
                if (!paused) yolo.resetStatistics();  // the pause would distort the averages
            }

            Yolo26Processor.Result result = yolo.getLatestResult();
            telemetry.addLine("YOLO26 on the Control Hub CPU" + (paused ? "   [PAUSED]" : ""));
            telemetry.addData("Model", "%s (%d of %d)", modelName(yolo.getModelPath()), modelIndex + 1, models.size());
            telemetry.addData("Engine", "%s, %s", yolo.getEngineName(), System.getProperty("os.arch"));
            telemetry.addData("Network", "%.0f ms  (average %.0f ms)", result.inferenceMs, yolo.getAverageInferenceMs());
            telemetry.addData("Pre/post", "%.1f ms / %.1f ms", result.preprocessMs, result.postprocessMs);
            telemetry.addData("Speed", "%.1f YOLO frames/s  (camera pipeline %.1f FPS)",
                    yolo.getProcessingFps(), portal.getFps());
            if (loadError != null) telemetry.addData("Load error", loadError);
            if (yolo.getLastError() != null) telemetry.addData("Error", yolo.getLastError());

            List<YoloDetection> found = result.detections;
            telemetry.addData("Found", found.size());
            for (int i = 0; i < Math.min(found.size(), MAX_LISTED_DETECTIONS); i++) {
                YoloDetection d = found.get(i);
                telemetry.addLine(String.format(Locale.US, "  %-6s %3.0f%%  center (%3.0f,%3.0f)  %3.0fx%-3.0f px  %+5.1f deg",
                        d.label, d.confidence * 100, d.centerX(), d.centerY(), d.width(), d.height(),
                        d.horizontalAngleDeg));
            }

            if (!speedResults.isEmpty()) {
                telemetry.addLine();
                telemetry.addLine("Speed results (network average, YOLO fps):");
                for (Map.Entry<String, String> entry : speedResults.entrySet()) {
                    telemetry.addLine("  " + entry.getKey() + " " + entry.getValue());
                }
            }
            telemetry.addLine();
            telemetry.addLine("dpad <> model | dpad ^v threads (" + threads + ") | A pause");
            telemetry.update();
            sleep(50);
        }

        RobotLog.ii("ConceptYolo26", "Speed results: %s", speedResults);
        if (dashboard != null) dashboard.stopCameraStream();
        portal.close();
        yolo.close();
    }

    /** The built-in models, then the .tflite and .onnx files in EXTRA_MODELS_FOLDER (alphabetical). */
    private static List<String> modelList() {
        List<String> models = new ArrayList<>(Arrays.asList(MODELS));
        File[] files = EXTRA_MODELS_FOLDER.listFiles();
        if (files != null) {
            Arrays.sort(files);
            for (File f : files) {
                String name = f.getName().toLowerCase(Locale.US);
                if (f.isFile() && (name.endsWith(".tflite") || name.endsWith(".onnx"))) {
                    models.add(f.getAbsolutePath());  // Yolo26Processor reads absolute paths from storage
                }
            }
        }
        return models;
    }

    /** "yolo/balls26n_256x192_int8.tflite" -> "balls26n_256x192_int8"; .onnx files get " onnx". */
    private static String modelName(String path) {
        String name = path.substring(path.lastIndexOf('/') + 1);
        if (name.endsWith(".tflite")) return name.substring(0, name.length() - ".tflite".length());
        int dot = name.lastIndexOf('.');
        return dot > 0 ? name.substring(0, dot) + " " + name.substring(dot + 1) : name;
    }
}
