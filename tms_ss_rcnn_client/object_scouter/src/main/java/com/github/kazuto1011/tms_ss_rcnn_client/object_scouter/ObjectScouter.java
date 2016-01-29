package com.github.kazuto1011.tms_ss_rcnn_client.object_scouter;

import android.content.Context;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.android.view.camera.RosCameraPreviewView;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class ObjectScouter extends RosActivity
{
    private final String TAG  = "Object Scouter";
    private ObjectDetectionClient objectDetectionClient;
    private RosCameraPreviewView rosCameraPreviewView;
    private Handler handler;
    private Camera camera;
    private int cameraId = 0;
    private Context context = this;

    public ObjectScouter() {
        super("Object Scouter", "Object Scouter");
    }

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.main);

        rosCameraPreviewView = (RosCameraPreviewView)findViewById(R.id.camera_view);

        handler = new Handler() {
            @Override
            public void handleMessage(Message msg) {
                super.handleMessage(msg);
                Toast.makeText(context, (String) msg.obj, Toast.LENGTH_LONG).show();
            }
        };

    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        camera = Camera.open(cameraId);

        /*
        Camera.Parameters params = camera.getParameters();
        params.setPictureSize(640 / 2, 480 / 2);
        params.setPreviewSize(640 / 2, 480 / 2);
        camera.setParameters(params);
        */

        int fps[] = new int[2];
        camera.getParameters().getPreviewFpsRange(fps);

        String info = "";
        info += "Frame rate: " + String.valueOf(fps[1] / 1000);
        info += "\nWidth: " + camera.getParameters().getPreviewSize().width;
        info += "\nHeight: " + camera.getParameters().getPreviewSize().height;
        info += "\nWidth: " + rosCameraPreviewView.getWidth();
        info += "\nHeight: " + rosCameraPreviewView.getHeight();

        Message msg = handler.obtainMessage(0, info);
        handler.sendMessage(msg);

        rosCameraPreviewView.setCamera(camera);

        objectDetectionClient = new ObjectDetectionClient();

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        nodeMainExecutor.execute(rosCameraPreviewView, nodeConfiguration.setNodeName("ObjectScouter/RosCameraPreviewView"));
        nodeMainExecutor.execute(objectDetectionClient, nodeConfiguration);
    }
}
