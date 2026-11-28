package org.firstinspires.ftc.teamcode.vision.yolo;

import org.opencv.core.Mat;

import java.io.Closeable;
import java.util.Locale;

/**
 * A YOLO26 model loaded into an inference engine that runs on the Control Hub's CPU.
 *
 * <ul>
 *   <li>{@code .tflite} files run on LiteRT (TensorFlow Lite) with XNNPACK: the fastest option.
 *       FP32 and INT8 models both work; INT8 is about twice as fast.</li>
 *   <li>{@code .onnx} files run on the OpenCV 4.10 DNN module that ships inside the FTC SDK, so they
 *       work with no extra library (slower; useful as a fallback and for comparison).</li>
 * </ul>
 *
 * <p>The input can be square (320x320) or camera-shaped (256x192 for a 4:3 webcam).
 */
public interface YoloModel extends Closeable {

    /** Width, in pixels, of the image the model expects. */
    int inputWidth();

    /** Height, in pixels, of the image the model expects. */
    int inputHeight();

    /** Output rows (anchors, or max detections for end-to-end exports). */
    int outputRows();

    /** Output columns (4 box values + class scores, or 6 for end-to-end exports). */
    int outputCols();

    /**
     * Runs the model.
     *
     * @param rgbInput letterboxed 8-bit RGB image, inputWidth x inputHeight
     * @return the output as a row-major [outputRows x outputCols] array. The array is reused by the
     *         next call, so read it before calling run() again.
     */
    float[] run(Mat rgbInput);

    /** Short description for telemetry, e.g. "LiteRT 1.4.2, XNNPACK, 4 threads". */
    String engineName();

    /** Frees the model's native memory. */
    @Override
    void close();

    /**
     * Loads a model, picking the engine from the file name.
     *
     * @param inputWidth  input width for .onnx models (.tflite models report their own size)
     * @param inputHeight input height for .onnx models
     * @param numThreads  CPU threads to use for inference; the Control Hub has 4 cores
     */
    static YoloModel create(byte[] modelBytes, String fileName, int inputWidth, int inputHeight, int numThreads) {
        String name = fileName.toLowerCase(Locale.US);
        if (name.endsWith(".tflite")) return new LiteRtYoloModel(modelBytes, numThreads);
        if (name.endsWith(".onnx")) return new OpenCvDnnYoloModel(modelBytes, inputWidth, inputHeight, numThreads);
        throw new IllegalArgumentException("Unsupported model file (use .tflite or .onnx): " + fileName);
    }
}
