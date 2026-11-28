package org.firstinspires.ftc.teamcode.vision.yolo;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * Turns the raw YOLO26 output into {@link YoloDetection}s.
 *
 * <p>YOLO26 is "NMS-free": its one-to-one head predicts each object once, so there is no
 * non-maximum-suppression step. Decoding is just "keep the rows whose score is high enough".
 */
final class YoloDecoder {

    /** The two output layouts this decoder understands. */
    enum Layout {
        /**
         * export_for_control_hub.py (default): [1, anchors, 4 + classes]. Each row is
         * x1, y1, x2, y2 (model-input pixels) followed by one 0..1 score per class.
         */
        RAW_SCORES,
        /**
         * Plain Ultralytics "nms=False" export (LiteRT only; OpenCV 4.10 cannot load it):
         * [1, maxDet, 6]. Each row is x1, y1, x2, y2, score, classId.
         */
        END_TO_END
    }

    /** Camera intrinsics, used to turn pixel positions into angles. */
    static final class Camera {
        final double fx, fy, cx, cy;

        Camera(double fx, double fy, double cx, double cy) {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
        }

        /** Pinhole approximation when the SDK has no calibration for this camera/resolution. */
        static Camera fromHorizontalFov(int width, int height, double horizontalFovDeg) {
            double f = (width / 2.0) / Math.tan(Math.toRadians(horizontalFovDeg) / 2.0);
            return new Camera(f, f, width / 2.0, height / 2.0);
        }
    }

    private static final Comparator<YoloDetection> BY_CONFIDENCE = new Comparator<YoloDetection>() {
        @Override public int compare(YoloDetection a, YoloDetection b) {
            return Float.compare(b.confidence, a.confidence);
        }
    };

    private YoloDecoder() { }

    /** Number of anchor points (output rows) of a stride 8/16/32 YOLO detector for this input size. */
    static int anchorCount(int inputWidth, int inputHeight) {
        int count = 0;
        for (int stride = 8; stride <= 32; stride *= 2) count += (inputWidth / stride) * (inputHeight / stride);
        return count;
    }

    /** Works out the output layout, or explains how to fix a model exported the wrong way. */
    static Layout layoutOf(int rows, int cols, int inputWidth, int inputHeight) {
        int anchors = anchorCount(inputWidth, inputHeight);
        if (rows == anchors && cols >= 5) return Layout.RAW_SCORES;
        if (cols == 6 && rows != anchors) return Layout.END_TO_END;
        if (cols == anchors) {
            throw new IllegalArgumentException(String.format(Locale.US,
                    "Model output is [1, %d, %d]: that is the classic YOLO layout, which needs NMS. "
                            + "Re-export the model with export_for_control_hub.py.", rows, cols));
        }
        throw new IllegalArgumentException(String.format(Locale.US,
                "Unexpected model output [1, %d, %d] for a %dx%d input (expected [1, %d, 4 + classes]). "
                        + "Check the input size, or re-export with export_for_control_hub.py.",
                rows, cols, inputWidth, inputHeight, anchors));
    }

    /**
     * @param output        raw model output, row-major [rows x cols]
     * @param minConfidence keep detections scoring at least this (0..1)
     * @param maxDetections keep at most this many (highest confidence first)
     * @param labels        class names (may be null or short; missing names become "class N")
     */
    static List<YoloDetection> decode(float[] output, int rows, int cols, Layout layout, Letterbox box,
                                      float minConfidence, int maxDetections, String[] labels, Camera camera) {
        ArrayList<YoloDetection> found = new ArrayList<>();
        int numClasses = cols - 4;
        for (int i = 0, base = 0; i < rows; i++, base += cols) {
            int classId;
            float score;
            if (layout == Layout.RAW_SCORES) {
                classId = 0;
                score = output[base + 4];
                for (int c = 1; c < numClasses; c++) {
                    float s = output[base + 4 + c];
                    if (s > score) {
                        score = s;
                        classId = c;
                    }
                }
            } else {
                score = output[base + 4];
                classId = Math.round(output[base + 5]);
            }
            if (!(score >= minConfidence)) continue;  // also drops NaN

            double x1 = clamp(box.toFrameX(output[base]), box.frameWidth);
            double y1 = clamp(box.toFrameY(output[base + 1]), box.frameHeight);
            double x2 = clamp(box.toFrameX(output[base + 2]), box.frameWidth);
            double y2 = clamp(box.toFrameY(output[base + 3]), box.frameHeight);
            if (x2 - x1 < 1 || y2 - y1 < 1) continue;  // box lies in the padding only

            double centerX = (x1 + x2) / 2, centerY = (y1 + y2) / 2;
            found.add(new YoloDetection(classId, labelFor(labels, classId), score,
                    (float) x1, (float) y1, (float) x2, (float) y2,
                    Math.toDegrees(Math.atan2(centerX - camera.cx, camera.fx)),
                    Math.toDegrees(Math.atan2(centerY - camera.cy, camera.fy))));
        }
        Collections.sort(found, BY_CONFIDENCE);
        if (found.size() > maxDetections) found.subList(maxDetections, found.size()).clear();
        return Collections.unmodifiableList(found);
    }

    static String labelFor(String[] labels, int classId) {
        if (labels != null && classId >= 0 && classId < labels.length && labels[classId] != null) {
            return labels[classId];
        }
        return "class " + classId;
    }

    private static double clamp(double v, double max) {
        return v < 0 ? 0 : (v > max ? max : v);
    }
}
