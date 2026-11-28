package org.firstinspires.ftc.teamcode.vision.yolo;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.Tensor;
import org.tensorflow.lite.TensorFlowLite;

import java.nio.Buffer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.Locale;

/**
 * Runs a .tflite YOLO26 model on LiteRT (Google's TensorFlow Lite runtime) on the CPU. LiteRT uses
 * XNNPACK, whose ARM NEON kernels are the fastest CPU option on the Control Hub's Cortex-A53 cores.
 *
 * <p>Needs {@code implementation 'com.google.ai.edge.litert:litert:1.4.2'} in TeamCode/build.gradle.
 */
final class LiteRtYoloModel implements YoloModel {

    private static final Scalar NO_MEAN = new Scalar(0, 0, 0);

    private final Interpreter interpreter;
    private final boolean channelsFirst;  // NCHW input (current Ultralytics LiteRT exports) vs NHWC
    private final int inputWidth, inputHeight, rows, cols;
    private final float[] inputValues, output;
    private final ByteBuffer inputBuffer, outputBuffer;
    private final Mat floatImage = new Mat();
    private final String engineName;

    LiteRtYoloModel(byte[] tfliteBytes, int numThreads) {
        ByteBuffer model = ByteBuffer.allocateDirect(tfliteBytes.length).order(ByteOrder.nativeOrder());
        model.put(tfliteBytes);
        ((Buffer) model).rewind();  // cast: ByteBuffer.rewind() does not exist on Android 7

        Interpreter.Options options = new Interpreter.Options();
        if (numThreads > 0) options.setNumThreads(numThreads);
        options.setUseXNNPACK(true);
        interpreter = new Interpreter(model, options);

        // INT8 models from export_for_control_hub.py keep float32 input and output tensors too.
        Tensor in = interpreter.getInputTensor(0);
        int[] s = in.shape();
        if (in.dataType() != DataType.FLOAT32 || s.length != 4 || s[0] != 1) {
            throw new IllegalArgumentException("Expected a float32 [1,3,H,W] or [1,H,W,3] input, got "
                    + in.dataType() + " " + Arrays.toString(s));
        }
        if (s[1] == 3) {
            channelsFirst = true;
            inputHeight = s[2];
            inputWidth = s[3];
        } else if (s[3] == 3) {
            channelsFirst = false;
            inputHeight = s[1];
            inputWidth = s[2];
        } else {
            throw new IllegalArgumentException("Expected an RGB (3-channel) input, got " + Arrays.toString(s));
        }

        Tensor out = interpreter.getOutputTensor(0);
        int[] o = out.shape();
        if (out.dataType() != DataType.FLOAT32 || o.length != 3 || o[0] != 1) {
            throw new IllegalArgumentException("Expected a float32 [1,N,C] output, got "
                    + out.dataType() + " " + Arrays.toString(o));
        }
        rows = o[1];
        cols = o[2];

        inputValues = new float[3 * inputWidth * inputHeight];
        output = new float[rows * cols];
        inputBuffer = ByteBuffer.allocateDirect(4 * inputValues.length).order(ByteOrder.nativeOrder());
        outputBuffer = ByteBuffer.allocateDirect(4 * output.length).order(ByteOrder.nativeOrder());

        String version;
        try {
            version = TensorFlowLite.runtimeVersion();
        } catch (Throwable t) {
            version = "?";
        }
        engineName = String.format(Locale.US, "LiteRT %s, XNNPACK, %s threads", version,
                numThreads > 0 ? String.valueOf(numThreads) : "auto");

        // Warm-up run: XNNPACK prepares its weights on the first run, which is much slower.
        Mat gray = new Mat(inputHeight, inputWidth, CvType.CV_8UC3, Letterbox.PAD_COLOR);
        try {
            run(gray);
        } finally {
            gray.release();
        }
    }

    @Override public int inputWidth()  { return inputWidth; }
    @Override public int inputHeight() { return inputHeight; }
    @Override public int outputRows() { return rows; }
    @Override public int outputCols() { return cols; }
    @Override public String engineName() { return engineName; }

    @Override
    public float[] run(Mat rgbInput) {
        if (channelsFirst) {
            // OpenCV's blobFromImage does HWC -> CHW and the 0..1 scaling in one optimized pass.
            Mat blob = Dnn.blobFromImage(rgbInput, 1.0 / 255.0, new Size(inputWidth, inputHeight), NO_MEAN, false, false);
            try {
                blob.get(new int[] {0, 0, 0, 0}, inputValues);
            } finally {
                blob.release();
            }
        } else {
            rgbInput.convertTo(floatImage, CvType.CV_32FC3, 1.0 / 255.0);
            floatImage.get(0, 0, inputValues);
        }
        ((Buffer) inputBuffer).clear();
        inputBuffer.asFloatBuffer().put(inputValues);
        ((Buffer) outputBuffer).clear();
        interpreter.run(inputBuffer, outputBuffer);
        ((Buffer) outputBuffer).clear();  // back to position 0 (clear() keeps the data)
        outputBuffer.asFloatBuffer().get(output);
        return output;
    }

    @Override
    public void close() {
        interpreter.close();
        floatImage.release();
    }
}
