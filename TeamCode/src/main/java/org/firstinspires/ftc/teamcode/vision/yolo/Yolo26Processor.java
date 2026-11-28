package org.firstinspires.ftc.teamcode.vision.yolo;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.io.ByteArrayOutputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Properties;

/**
 * VisionPortal processor that runs an Ultralytics YOLO26 detector on the Control Hub's CPU.
 *
 * <pre>
 * Yolo26Processor yolo = new Yolo26Processor.Builder()
 *         .setModel("yolo/balls26n_256x192_int8.tflite")   // from TeamCode/src/main/assets
 *         .build();
 * VisionPortal portal = new VisionPortal.Builder()
 *         .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
 *         .addProcessor(yolo)
 *         .build();
 * ...
 * for (YoloDetection d : yolo.getDetections()) { ... }
 * </pre>
 *
 * Inference runs on the camera thread, one frame at a time: while the model is busy, newer frames
 * are skipped. Boxes drawn on the camera stream therefore always line up with the frame they were
 * found in.
 */
public class Yolo26Processor implements VisionProcessor {

    private static final String TAG = "Yolo26";

    /** Horizontal field of view used for angles when the SDK has no calibration for the camera. */
    private static final double FALLBACK_HFOV_DEG = 42.5;  // Logitech C270 at 640x480

    public static class Builder {
        private String modelPath = "yolo/balls26n_256x192_int8.tflite";
        private String[] labels = null;
        private int inputWidth = 0, inputHeight = 0;
        private float minConfidence = 0.40f;
        private int maxDetections = 20;
        private int numThreads = 4;
        private boolean drawDetections = true;

        /**
         * Model to load: a file in TeamCode/src/main/assets (for example
         * "yolo/balls26n_256x192_int8.tflite"), or an absolute path on the Control Hub (for example
         * "/sdcard/FIRST/yolo/balls26n_256x192_int8.tflite"). A .tflite file runs on LiteRT
         * (fastest); an .onnx file runs on the SDK's OpenCV DNN.
         */
        public Builder setModel(String modelPath) {
            this.modelPath = modelPath;
            return this;
        }

        /** Class names in class-id order. Default: read from the model's .properties file. */
        public Builder setLabels(String... labels) {
            this.labels = labels.clone();
            return this;
        }

        /** Square model input size in pixels. Default: from the model or its .properties file. */
        public Builder setInputSize(int pixels) {
            return setInputSize(pixels, pixels);
        }

        /**
         * Model input size in pixels, for example 256 x 192. Only needed for an .onnx model without
         * its .properties file: .tflite models know their own size.
         */
        public Builder setInputSize(int width, int height) {
            this.inputWidth = width;
            this.inputHeight = height;
            return this;
        }

        /** Ignore detections below this confidence (0..1). Default 0.40. */
        public Builder setMinConfidence(float minConfidence) {
            this.minConfidence = minConfidence;
            return this;
        }

        /** Report at most this many detections per frame, highest confidence first. Default 20. */
        public Builder setMaxDetections(int maxDetections) {
            this.maxDetections = maxDetections;
            return this;
        }

        /** CPU threads used for inference. The Control Hub has 4 cores. Default 4. */
        public Builder setNumThreads(int numThreads) {
            this.numThreads = numThreads;
            return this;
        }

        /** Draw boxes and labels on the camera stream / live view. Default true. */
        public Builder setDrawDetections(boolean drawDetections) {
            this.drawDetections = drawDetections;
            return this;
        }

        /**
         * Loads the model right away, so a missing or broken model is reported during INIT.
         *
         * @throws IllegalStateException if the model can't be loaded
         */
        public Yolo26Processor build() {
            return new Yolo26Processor(this);
        }
    }

    /** Everything YOLO found in one camera frame, plus how long each step took. */
    public static final class Result {
        static final Result EMPTY = new Result(Collections.<YoloDetection>emptyList(), 0, 0, 0, 0);

        /** Detections, highest confidence first. Never null. */
        public final List<YoloDetection> detections;
        /** When the camera captured the frame (System.nanoTime() time base), 0 if none yet. */
        public final long frameTimeNanos;
        /** Resize, color conversion and padding. */
        public final double preprocessMs;
        /** The neural network itself. */
        public final double inferenceMs;
        /** Turning the network output into detections. */
        public final double postprocessMs;

        Result(List<YoloDetection> detections, long frameTimeNanos,
               double preprocessMs, double inferenceMs, double postprocessMs) {
            this.detections = detections;
            this.frameTimeNanos = frameTimeNanos;
            this.preprocessMs = preprocessMs;
            this.inferenceMs = inferenceMs;
            this.postprocessMs = postprocessMs;
        }

        public double totalMs() { return preprocessMs + inferenceMs + postprocessMs; }
    }

    private static final class LoadedModel {
        final YoloModel model;
        final String[] labels;
        final YoloDecoder.Layout layout;

        LoadedModel(YoloModel model, String[] labels, YoloDecoder.Layout layout) {
            this.model = model;
            this.labels = labels;
            this.layout = layout;
        }
    }

    // Model state. processFrame() holds modelLock while it runs, so swapping models is safe.
    private final Object modelLock = new Object();
    private YoloModel model;
    private String[] labels;
    private YoloDecoder.Layout layout;
    private Letterbox letterbox;
    private final Mat scaled = new Mat(), rgb = new Mat(), input = new Mat();

    private final String[] userLabels;
    private final int userInputWidth, userInputHeight;
    private final int maxDetections;
    private final boolean drawDetections;
    private volatile float minConfidence;
    private volatile YoloDecoder.Camera camera;

    // Results and statistics, readable from the OpMode thread.
    private volatile Result latest = Result.EMPTY;
    private volatile String modelPath;
    private volatile String engineName;
    private volatile int inputWidth, inputHeight;
    private volatile String lastError;
    private volatile double avgInferenceMs;
    private volatile double avgFrameIntervalMs;
    private volatile int frameCount;
    private long lastFrameDoneNanos;

    // Drawing.
    private final Object drawLock = new Object();
    private final Paint boxPaint = new Paint();
    private final Paint textPaint = new Paint();
    private final Paint textBackgroundPaint = new Paint();

    private Yolo26Processor(Builder builder) {
        userLabels = builder.labels;
        userInputWidth = builder.inputWidth;
        userInputHeight = builder.inputHeight;
        maxDetections = Math.max(1, builder.maxDetections);
        drawDetections = builder.drawDetections;
        minConfidence = builder.minConfidence;

        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setColor(Color.MAGENTA);  // stands out against yellow balls
        boxPaint.setAntiAlias(true);
        textPaint.setColor(Color.WHITE);
        textPaint.setAntiAlias(true);
        textBackgroundPaint.setColor(Color.argb(170, 0, 0, 0));

        try {
            setModel(builder.modelPath, builder.numThreads);
        } catch (IOException e) {
            throw new IllegalStateException(e.getMessage(), e);
        }
    }

    /**
     * Loads another model (and/or thread count) while the camera keeps running. If the new model
     * can't be loaded, the current one stays active and an IOException explains why.
     */
    public void setModel(String modelPath, int numThreads) throws IOException {
        LoadedModel loaded = load(modelPath, userLabels, userInputWidth, userInputHeight, numThreads);  // slow: no lock yet
        YoloModel previous;
        synchronized (modelLock) {
            previous = model;
            model = loaded.model;
            labels = loaded.labels;
            layout = loaded.layout;
            letterbox = null;
            this.modelPath = modelPath;
            engineName = loaded.model.engineName();
            inputWidth = loaded.model.inputWidth();
            inputHeight = loaded.model.inputHeight();
            latest = Result.EMPTY;
            lastError = null;
            avgInferenceMs = 0;
            avgFrameIntervalMs = 0;
            frameCount = 0;
            lastFrameDoneNanos = 0;
        }
        if (previous != null) previous.close();  // processFrame() can no longer reach it
        RobotLog.ii(TAG, "Loaded %s (%s): input %dx%d, output [1, %d, %d], labels %s",
                modelPath, loaded.model.engineName(), loaded.model.inputWidth(), loaded.model.inputHeight(),
                loaded.model.outputRows(), loaded.model.outputCols(), Arrays.toString(loaded.labels));
    }

    /** Latest detections, highest confidence first. Never null. */
    public List<YoloDetection> getDetections() { return latest.detections; }

    /** Latest detections plus timing for that frame. */
    public Result getLatestResult() { return latest; }

    /** Inference time, averaged over recent frames. */
    public double getAverageInferenceMs() { return avgInferenceMs; }

    /** How many frames per second YOLO is getting through. */
    public double getProcessingFps() {
        double interval = avgFrameIntervalMs;
        return interval > 0 ? 1000.0 / interval : 0;
    }

    /** Frames processed since the current model was loaded (the averages above settle after ~30). */
    public int getFrameCount() { return frameCount; }

    public String getModelPath() { return modelPath; }

    public String getEngineName() { return engineName; }

    /** Model input width in pixels, for example 256. */
    public int getInputWidth() { return inputWidth; }

    /** Model input height in pixels, for example 192. */
    public int getInputHeight() { return inputHeight; }

    /** The last error inside processFrame(), or null. */
    public String getLastError() { return lastError; }

    public void setMinConfidence(float minConfidence) { this.minConfidence = minConfidence; }

    /** Restarts the timing averages and the frame count, for example after pausing the processor. */
    public void resetStatistics() {
        synchronized (modelLock) {
            avgInferenceMs = 0;
            avgFrameIntervalMs = 0;
            frameCount = 0;
            lastFrameDoneNanos = 0;
        }
    }

    /** Frees the model. Call after closing the VisionPortal. */
    public void close() {
        synchronized (modelLock) {
            if (model != null) model.close();
            model = null;
            scaled.release();
            rgb.release();
            input.release();
        }
    }

    // ---------------------------------------------------------------------------------------------
    // VisionProcessor
    // ---------------------------------------------------------------------------------------------

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        if (calibration != null && !calibration.isDegenerate() && calibration.focalLengthX > 0) {
            camera = new YoloDecoder.Camera(calibration.focalLengthX, calibration.focalLengthY,
                    calibration.principalPointX, calibration.principalPointY);
        } else {
            camera = YoloDecoder.Camera.fromHorizontalFov(width, height, FALLBACK_HFOV_DEG);
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        synchronized (modelLock) {
            if (model == null) return null;
            try {
                long t0 = System.nanoTime();
                if (letterbox == null
                        || !letterbox.matches(frame.cols(), frame.rows(), model.inputWidth(), model.inputHeight())) {
                    letterbox = new Letterbox(frame.cols(), frame.rows(), model.inputWidth(), model.inputHeight());
                }
                letterbox.apply(frame, scaled, rgb, input);
                long t1 = System.nanoTime();
                float[] output = model.run(input);
                long t2 = System.nanoTime();
                YoloDecoder.Camera cam = camera != null ? camera
                        : YoloDecoder.Camera.fromHorizontalFov(frame.cols(), frame.rows(), FALLBACK_HFOV_DEG);
                List<YoloDetection> detections = YoloDecoder.decode(output, model.outputRows(), model.outputCols(),
                        layout, letterbox, minConfidence, maxDetections, labels, cam);
                long t3 = System.nanoTime();

                Result result = new Result(detections, captureTimeNanos, ms(t1 - t0), ms(t2 - t1), ms(t3 - t2));
                avgInferenceMs = smooth(avgInferenceMs, result.inferenceMs);
                if (lastFrameDoneNanos != 0) avgFrameIntervalMs = smooth(avgFrameIntervalMs, ms(t3 - lastFrameDoneNanos));
                lastFrameDoneNanos = t3;
                frameCount++;
                latest = result;
                return result;
            } catch (RuntimeException e) {
                // Keep the camera running; show the problem in telemetry instead of crashing the OpMode.
                if (lastError == null) RobotLog.ee(TAG, e, "YOLO processing failed");
                lastError = e.toString();
                return null;
            }
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (!drawDetections || !(userContext instanceof Result)) return;
        Result result = (Result) userContext;
        synchronized (drawLock) {
            boxPaint.setStrokeWidth(3 * scaleCanvasDensity);
            textPaint.setTextSize(16 * scaleCanvasDensity);
            float pad = 3 * scaleCanvasDensity;
            float textHeight = textPaint.getTextSize();

            for (YoloDetection d : result.detections) {
                float left = d.left * scaleBmpPxToCanvasPx, top = d.top * scaleBmpPxToCanvasPx;
                float right = d.right * scaleBmpPxToCanvasPx, bottom = d.bottom * scaleBmpPxToCanvasPx;
                canvas.drawRect(left, top, right, bottom, boxPaint);

                String text = String.format(Locale.US, "%s %.0f%%", d.label, d.confidence * 100);
                float baseline = top - pad > textHeight ? top - pad : bottom + textHeight;  // above the box if it fits
                canvas.drawRect(left, baseline - textHeight, left + textPaint.measureText(text) + 2 * pad,
                        baseline + pad, textBackgroundPaint);
                canvas.drawText(text, left + pad, baseline, textPaint);
            }

            String status = String.format(Locale.US, "YOLO26 %.0f ms | %d found", result.inferenceMs, result.detections.size());
            canvas.drawRect(0, 0, textPaint.measureText(status) + 2 * pad, textHeight + 2 * pad, textBackgroundPaint);
            canvas.drawText(status, pad, textHeight, textPaint);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Model loading
    // ---------------------------------------------------------------------------------------------

    private static LoadedModel load(String modelPath, String[] userLabels, int userInputWidth, int userInputHeight,
                                    int numThreads) throws IOException {
        Properties info = readProperties(siblingPath(modelPath, ".properties"));
        // input_width/input_height, or imgsz (square) in .properties files from older exports
        int square = parseInt(info.getProperty("imgsz"), 320);
        int width = userInputWidth > 0 ? userInputWidth : parseInt(info.getProperty("input_width"), square);
        int height = userInputHeight > 0 ? userInputHeight : parseInt(info.getProperty("input_height"), square);
        byte[] bytes = readBytes(modelPath);

        YoloModel model;
        try {
            model = YoloModel.create(bytes, modelPath, width, height, numThreads);
        } catch (RuntimeException | LinkageError e) {  // LinkageError: e.g. native library missing
            throw new IOException("Could not load " + modelPath + ": " + e, e);
        }

        YoloDecoder.Layout layout;
        try {
            layout = YoloDecoder.layoutOf(model.outputRows(), model.outputCols(), model.inputWidth(), model.inputHeight());
        } catch (IllegalArgumentException e) {
            model.close();
            throw new IOException(modelPath + ": " + e.getMessage(), e);
        }

        String[] labels = userLabels;
        if (labels == null) {
            int numClasses = layout == YoloDecoder.Layout.RAW_SCORES ? model.outputCols() - 4
                    : parseInt(info.getProperty("num_classes"), 0);
            labels = new String[numClasses];
            for (int i = 0; i < numClasses; i++) labels[i] = info.getProperty("label." + i, "class " + i);
        }
        return new LoadedModel(model, labels, layout);
    }

    /** "yolo/balls26n_256x192_int8.tflite" -> "yolo/balls26n_256x192_int8.properties" */
    private static String siblingPath(String path, String newExtension) {
        int dot = path.lastIndexOf('.');
        int slash = path.lastIndexOf('/');
        return (dot > slash ? path.substring(0, dot) : path) + newExtension;
    }

    private static InputStream open(String path) throws IOException {
        if (path.startsWith("/")) return new FileInputStream(path);  // file on the Control Hub
        return AppUtil.getDefContext().getAssets().open(path);       // asset packaged in the APK
    }

    private static byte[] readBytes(String path) throws IOException {
        InputStream in;
        try {
            in = open(path);
        } catch (FileNotFoundException e) {
            throw new IOException("Model not found: " + path
                    + " (assets live in TeamCode/src/main/assets; absolute paths are files on the Control Hub)", e);
        }
        try {
            ByteArrayOutputStream out = new ByteArrayOutputStream(1 << 20);
            byte[] chunk = new byte[1 << 16];
            int n;
            while ((n = in.read(chunk)) > 0) out.write(chunk, 0, n);
            return out.toByteArray();
        } finally {
            in.close();
        }
    }

    private static Properties readProperties(String path) {
        Properties properties = new Properties();
        try {
            InputStream in = open(path);
            try {
                properties.load(in);
            } finally {
                in.close();
            }
        } catch (IOException e) {
            // Optional file: labels default to "class N" and the input size to 320.
        }
        return properties;
    }

    private static int parseInt(String text, int fallback) {
        try {
            return text == null ? fallback : Integer.parseInt(text.trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static double ms(long nanos) { return nanos / 1e6; }

    private static double smooth(double average, double sample) {
        return average == 0 ? sample : 0.9 * average + 0.1 * sample;
    }
}
