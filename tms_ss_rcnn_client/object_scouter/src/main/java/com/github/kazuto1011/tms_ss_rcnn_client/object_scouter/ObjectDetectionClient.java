package com.github.kazuto1011.tms_ss_rcnn_client.object_scouter;

import android.content.Context;
import android.os.Handler;
import android.util.Log;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.jboss.netty.handler.codec.marshalling.ContextBoundUnmarshallerProvider;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.Highgui;
import org.ros.address.InetAddressFactory;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.android.RosActivity;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.io.IOException;
import java.net.URI;
import java.util.List;

import tms_msg_db.Tmsdb;
import tms_ss_rcnn.obj_detectionRequest;
import tms_ss_rcnn.obj_detectionResponse;
import tms_ss_rcnn.object;

import static org.opencv.core.Core.putText;
import static org.opencv.core.Core.rectangle;


public class ObjectDetectionClient extends AbstractNodeMain {
    private String TAG = "ObjectDetectionClient";

    private ServiceClient<obj_detectionRequest, obj_detectionResponse> serviceClient;

    private ConnectedNode mConnectedNode;
    private int sequenceNumber = 0;
    private Handler handler;
    private Context context;

    public ObjectDetectionClient(Handler handler, Context context) {
        this.handler = handler;
        this.context = context;
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
        final List<Tmsdb> obj_infos = DbReaderClient.getTmsdbs();
        serviceClient.call(request, new ServiceResponseListener<obj_detectionResponse>() {
            @Override
            public void onSuccess(final obj_detectionResponse response) {
                Log.i(TAG, "Succeeded to call service");
                List<object> objects = response.getObjects();
                for (object obj : objects) {

                        int tl_x = obj.getRegion().getXOffset();
                        int tl_y = obj.getRegion().getYOffset();
                        int br_x = tl_x + obj.getRegion().getWidth();
                        int br_y = tl_y + obj.getRegion().getHeight();
                        rectangle(inputFrame, new Point(tl_x, tl_y), new Point(br_x, br_y), new Scalar(255, 255, 255), 2);
                        putText(inputFrame, "Houseware:" + obj.getClassName(), new Point(tl_x, tl_y + 30), Core.FONT_HERSHEY_COMPLEX, 1.0f, new Scalar(255, 0, 0), 2);

                        int i = obj_infos.get(0).getId();
                    /*
                    for (Tmsdb obj_info : obj_infos) {
                        int id = obj_info.getId();
                     *//*   String score = obj_info.getNote();
                        String time = obj_info.getTime();*//*
                        putText(inputFrame, "ID:" + id, new Point(tl_x, br_y + 30), Core.FONT_HERSHEY_COMPLEX, 1.0f, new Scalar(0, 0, 255), 2);
                 */
                }

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
