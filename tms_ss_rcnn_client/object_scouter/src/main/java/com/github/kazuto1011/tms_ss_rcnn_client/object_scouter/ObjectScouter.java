package com.github.kazuto1011.tms_ss_rcnn_client.object_scouter;

import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.widget.Toast;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.IOException;

import uk.co.chrisjenx.calligraphy.CalligraphyConfig;
import uk.co.chrisjenx.calligraphy.CalligraphyContextWrapper;

public class ObjectScouter extends RosActivity implements CameraBridgeViewBase.CvCameraViewListener2
{
    private final String TAG  = "Object Scouter";
    private ObjectDetectionClient objectDetectionClient;
    private Handler handler;
    private Context context = this;

    public ObjectScouter() {
        super("Object Scouter", "Object Scouter");
    }

    private CameraBridgeViewBase mCameraView;
    private Mat mOutputFrame;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                    mCameraView.enableView();
                    break;
                default:
                    super.onManagerConnected(status);
                    break;
            }
        }
    };

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);

        //change the font
        CalligraphyConfig.initDefault(new CalligraphyConfig.Builder()
                        .setDefaultFontPath("fonts/Roboto-Light.ttf")
                        .setFontAttrId(R.attr.fontPath)
                        .build()
        );

        setContentView(R.layout.main);

        mCameraView = (CameraBridgeViewBase)findViewById(R.id.camera_view);

        handler = new Handler() {
            @Override
            public void handleMessage(Message msg) {
                super.handleMessage(msg);
                Toast.makeText(context, (String) msg.obj, Toast.LENGTH_LONG).show();
            }
        };

        objectDetectionClient = new ObjectDetectionClient();
    }

    @Override
    protected void attachBaseContext(Context newBase) {
        super.attachBaseContext(CalligraphyContextWrapper.wrap(newBase));
    }

    @Override
    protected void onPause() {
        if (mCameraView != null) {
            mCameraView.disableView();
        }

        super.onPause();
    }

    @Override
    protected void onResume() {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_11, this, mLoaderCallback);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (mCameraView != null) {
            mCameraView.disableView();
        }
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        mOutputFrame = new Mat(height, width, CvType.CV_8UC3);
    }

    @Override
    public void onCameraViewStopped() {
        mOutputFrame.release();
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        try {
            mOutputFrame = objectDetectionClient.request(inputFrame.rgba());
        } catch (IOException e) {
            e.printStackTrace();
        }
        return mOutputFrame;
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        nodeMainExecutor.execute(objectDetectionClient, nodeConfiguration);

        mCameraView.setCvCameraViewListener(this);
    }
}
