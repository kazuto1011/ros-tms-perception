package com.github.kazuto1011.tms_perception.tms_ss_android.faster_rcnn_clinet;

import android.content.Context;
import android.hardware.Camera;
import android.util.AttributeSet;

import org.opencv.android.JavaCameraView;

/**
 * Created by common on 16/07/14.
 */
import java.util.List;

public class VariableJavaCameraView extends JavaCameraView {
    private static final String TAG = "VariableJavaCameraView";

    public VariableJavaCameraView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public List<Camera.Size> getResolutionList() {
        return mCamera.getParameters().getSupportedPreviewSizes();
    }

    public void setResolution(Camera.Size resolution) {
        disconnectCamera();
        mMaxHeight = resolution.height;
        mMaxWidth = resolution.width;
        connectCamera(getWidth(), getHeight());
    }

    public Camera.Size getResolution() {
        return mCamera.getParameters().getPreviewSize();
    }
}