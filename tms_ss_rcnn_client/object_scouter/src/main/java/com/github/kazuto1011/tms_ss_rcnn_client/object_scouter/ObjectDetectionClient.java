package com.github.kazuto1011.tms_ss_rcnn_client.object_scouter;

import android.os.Handler;
import android.util.Log;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Scalar;
import org.opencv.highgui.Highgui;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.io.IOException;

import tms_ss_rcnn.obj_detectionRequest;
import tms_ss_rcnn.obj_detectionResponse;


public class ObjectDetectionClient extends AbstractNodeMain {
    private String TAG = "ObjectDetectionClient";

    // Object detection client
    private ServiceClient<obj_detectionRequest, obj_detectionResponse> serviceClient;

    private ConnectedNode mConnectedNode;
    private int sequenceNumber = 0;
    private Handler handler;

    public ObjectDetectionClient(Handler handler) {
        this.handler = handler;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ObjectScouter/ObjectDetectionClient");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);


        try {
            serviceClient = connectedNode.newServiceClient("faster_rcnn", tms_ss_rcnn.obj_detection._TYPE);
        } catch (ServiceNotFoundException e) {
            throw new RosRuntimeException(e);
        }

        mConnectedNode = connectedNode;
    }

    private void setCompressedImage(Mat inputFrame, tms_ss_rcnn.obj_detectionRequest request) throws IOException {
        Time time = mConnectedNode.getCurrentTime();
        request.getImage().getHeader().setStamp(time);
        request.getImage().getHeader().setFrameId("ObjectScouter");
        request.getImage().getHeader().setSeq(sequenceNumber++);
        request.getImage().setFormat("jpg");

        MatOfByte buf = new MatOfByte();
        Highgui.imencode(".jpg", inputFrame, buf);
        ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        stream.write(buf.toArray());

        request.getImage().setData(stream.buffer().copy());
    }

    public Mat request(final Mat inputFrame) throws IOException {
        final tms_ss_rcnn.obj_detectionRequest request = serviceClient.newMessage();

        setCompressedImage(inputFrame, request);

        serviceClient.call(request, new ServiceResponseListener<obj_detectionResponse>() {
            @Override
            public void onSuccess(final obj_detectionResponse response) {
                Log.i(TAG, "Succeeded to call service");
            }

            @Override
            public void onFailure(RemoteException e) {
                Log.i(TAG, "Failed to call service");
                throw new RosRuntimeException(e);
            }
        });

        return inputFrame;
    }

    public Mat request_test(Mat inputFrame) {
        Mat outputFrame = new Mat();
        Core.absdiff(inputFrame, new Scalar(255, 255, 255), outputFrame);
        return outputFrame;
    }
}
