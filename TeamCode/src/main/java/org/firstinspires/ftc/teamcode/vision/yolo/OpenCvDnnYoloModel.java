package org.firstinspires.ftc.teamcode.vision.yolo;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;

import java.util.Locale;

/**
 * Runs an ONNX YOLO26 model on the OpenCV DNN module that is already part of the FTC SDK
 * (OpenCV 4.10), so it needs no extra library.
 *
 * <p>OpenCV 4.10 cannot run the TopK step at the end of a standard YOLO26 export, which is why
 * export_for_control_hub.py stops the graph just before it.
 */
final class OpenCvDnnYoloModel implements YoloModel {

    private static final Scalar NO_MEAN = new Scalar(0, 0, 0);

    private final Net net;
    private final int inputWidth, inputHeight;
    private final String engineName;
    private int rows, cols;
    private float[] output = new float[0];

    /** ONNX files don't say their input size, so it comes from the model's .properties file. */
    OpenCvDnnYoloModel(byte[] onnxBytes, int inputWidth, int inputHeight, int numThreads) {
        if (inputWidth <= 0 || inputHeight <= 0 || inputWidth % 32 != 0 || inputHeight % 32 != 0) {
            throw new IllegalArgumentException("ONNX models need an input width and height that are multiples of 32, got "
                    + inputWidth + "x" + inputHeight);
        }
        this.inputWidth = inputWidth;
        this.inputHeight = inputHeight;
        MatOfByte buffer = new MatOfByte(onnxBytes);
        try {
            net = Dnn.readNetFromONNX(buffer);
        } finally {
            buffer.release();
        }
        if (net.empty()) throw new IllegalStateException("OpenCV DNN could not read the ONNX model");
        net.setPreferableBackend(Dnn.DNN_BACKEND_OPENCV);
        net.setPreferableTarget(Dnn.DNN_TARGET_CPU);
        // Note: OpenCV has one thread pool for the whole app, so this also affects other OpenCV work.
        if (numThreads > 0) Core.setNumThreads(numThreads);
        engineName = String.format(Locale.US, "OpenCV %s DNN, %d threads", Core.VERSION, Core.getNumThreads());

        // Warm-up run: learns the output shape, and the first run is always the slowest.
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
        // NCHW float blob, scaled to 0..1. The input is already RGB, so no channel swap.
        Mat blob = Dnn.blobFromImage(rgbInput, 1.0 / 255.0, new Size(inputWidth, inputHeight), NO_MEAN, false, false);
        Mat out = null;
        try {
            net.setInput(blob);
            out = net.forward();
            if (out.dims() != 3 || out.size(0) != 1) {
                throw new IllegalStateException("Unexpected ONNX output with " + out.dims() + " dimensions");
            }
            rows = out.size(1);
            cols = out.size(2);
            if (output.length != rows * cols) output = new float[rows * cols];
            out.get(new int[] {0, 0, 0}, output);
            return output;
        } finally {
            blob.release();
            if (out != null) out.release();
        }
    }

    @Override
    public void close() {
        // Net has no explicit release(); dropping the reference lets its finalizer free the memory.
    }
}
