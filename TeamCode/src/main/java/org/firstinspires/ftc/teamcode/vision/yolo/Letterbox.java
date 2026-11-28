package org.firstinspires.ftc.teamcode.vision.yolo;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Fits a camera frame into YOLO's input without stretching it: shrink the frame until it fits,
 * then fill the leftover space with gray. The math matches Ultralytics' LetterBox (and
 * export_for_control_hub.py), so the robot feeds the model exactly what it was validated on.
 *
 * <p>Examples for a 640x480 frame: a 320x320 model gets a 320x240 image with 40 gray rows above and
 * below it; a 256x192 model (same 4:3 shape as the camera) gets a plain 256x192 resize, no gray.
 */
final class Letterbox {

    /** Gray used for padding (same value Ultralytics trains and validates with). */
    static final Scalar PAD_COLOR = new Scalar(114, 114, 114);

    final int frameWidth, frameHeight, inputWidth, inputHeight;
    final double scale;
    final int scaledWidth, scaledHeight;
    final int padLeft, padTop, padRight, padBottom;

    Letterbox(int frameWidth, int frameHeight, int inputWidth, int inputHeight) {
        this.frameWidth = frameWidth;
        this.frameHeight = frameHeight;
        this.inputWidth = inputWidth;
        this.inputHeight = inputHeight;
        scale = Math.min((double) inputHeight / frameHeight, (double) inputWidth / frameWidth);
        // Math.rint rounds half to even, like Python's round() in Ultralytics.
        scaledWidth = (int) Math.rint(frameWidth * scale);
        scaledHeight = (int) Math.rint(frameHeight * scale);
        double padX = (inputWidth - scaledWidth) / 2.0;
        double padY = (inputHeight - scaledHeight) / 2.0;
        padTop = (int) Math.rint(padY - 0.1);
        padBottom = (int) Math.rint(padY + 0.1);
        padLeft = (int) Math.rint(padX - 0.1);
        padRight = (int) Math.rint(padX + 0.1);
    }

    boolean matches(int frameWidth, int frameHeight, int inputWidth, int inputHeight) {
        return this.frameWidth == frameWidth && this.frameHeight == frameHeight
                && this.inputWidth == inputWidth && this.inputHeight == inputHeight;
    }

    /**
     * Turns a VisionPortal frame (RGBA, RGB or gray) into the model's inputWidth x inputHeight RGB image.
     *
     * @param scaled scratch Mat (reused between frames)
     * @param rgb    scratch Mat (reused between frames)
     * @param out    receives the letterboxed RGB image
     */
    void apply(Mat frame, Mat scaled, Mat rgb, Mat out) {
        // Shrink first, then convert colors: converting the small image is cheaper.
        Imgproc.resize(frame, scaled, new Size(scaledWidth, scaledHeight), 0, 0, Imgproc.INTER_LINEAR);
        switch (scaled.channels()) {
            case 4:  Imgproc.cvtColor(scaled, rgb, Imgproc.COLOR_RGBA2RGB); break;  // VisionPortal frames
            case 1:  Imgproc.cvtColor(scaled, rgb, Imgproc.COLOR_GRAY2RGB); break;
            default: scaled.copyTo(rgb); break;                                    // already RGB
        }
        Core.copyMakeBorder(rgb, out, padTop, padBottom, padLeft, padRight, Core.BORDER_CONSTANT, PAD_COLOR);
    }

    /** Converts an x coordinate in model-input pixels back to camera-frame pixels. */
    double toFrameX(double inputX) { return (inputX - padLeft) / scale; }

    /** Converts a y coordinate in model-input pixels back to camera-frame pixels. */
    double toFrameY(double inputY) { return (inputY - padTop) / scale; }
}
