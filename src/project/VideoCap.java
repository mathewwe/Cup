package project;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.opencv.videoio.Videoio;

import java.awt.image.BufferedImage;

public class VideoCap {

    static{
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    VideoCapture cap;
    Mat2Image mat2Img = new Mat2Image();

    VideoCap(int index){

        cap = new VideoCapture(Videoio.CAP_DSHOW + index);
        cap.set(Videoio.CAP_PROP_EXPOSURE, -7);
        cap.set(Videoio.CAP_PROP_FPS, 120);
        cap.set(Videoio.CAP_PROP_FRAME_WIDTH, 400);
        cap.set(Videoio.CAP_PROP_FRAME_HEIGHT, 400);
        /*
        cap.open(index);
        cap.set(Videoio.CAP_PROP_EXPOSURE, -6.9);
        cap.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M', 'J', 'P', 'G'));
        cap.set(Videoio.CAP_PROP_SETTINGS, 1);
        cap.set(Videoio.CAP_PROP_FPS, 120);
        cap.set(Videoio.CAP_PROP_FRAME_WIDTH, 630);
        cap.set(Videoio.CAP_PROP_FRAME_HEIGHT, 470);
        */
    }
    void read() {
    	cap.read(mat2Img.mat);
    }
    BufferedImage getOneFrame() {
        return mat2Img.getImage(mat2Img.mat);
    }
    Mat getCapturedMat() {
    	return mat2Img.mat;
    }
    BufferedImage getImage(Mat mat) {
    	return mat2Img.getImage(mat);
    }
}
