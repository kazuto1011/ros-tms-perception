package com.github.kazuto1011.tms_ss_rcnn_client.object_scouter;

import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.util.Log;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;

import sensor_msgs.CompressedImage;
import tms_ss_rcnn.obj_detectionRequest;
import tms_ss_rcnn.obj_detectionResponse;


public class ObjectDetectionClient extends AbstractNodeMain{
    private String TAG = "ObjectDetectionClient";
    private byte[] rawImageBuffer;
    private Camera.Size rawImageSize;
    private YuvImage yuvImage;
    private Rect rect;
    private ChannelBufferOutputStream stream  = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());

    // Message
    private sensor_msgs.CompressedImage img;

    // CompressedImage publisher
    private Publisher<CompressedImage> publisher;

    // Object detection client
    private ServiceClient<obj_detectionRequest, obj_detectionResponse> serviceClient;

    private Time time;
    private int sequenceNumber = 0;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ObjectScouter/ObjectDetectionClient");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        super.onStart(connectedNode);

        try {
            serviceClient = connectedNode.newServiceClient("faster_rcnn", tms_ss_rcnn.obj_detection._TYPE);
        } catch (ServiceNotFoundException e) {
            throw new RosRuntimeException(e);
        }

        NameResolver resolver = connectedNode.getResolver().newChild("android");
        publisher = connectedNode.newPublisher(resolver.resolve("camera/compressed"), sensor_msgs.CompressedImage._TYPE);

//        connectedNode.executeCancellableLoop(new CancellableLoop() {
//            @Override
//            protected void setup() {
//                Log.d("rosjava", "setup");
//                try {
//                    Thread.sleep(2000);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//            }
//
//            @Override
//            protected void loop() throws InterruptedException {
//                if (Preview.yuv_data != null) {
//                    if (Preview.yuv_data != rawImageBuffer || !Preview.mPreviewSize.equals(rawImageSize)) {
//                        rawImageBuffer = Preview.yuv_data;
//                        rawImageSize = Preview.mPreviewSize;
//                    }
//
//                    // new rosjava message
//                    img = publisher.newMessage();
//
//                    // message setting
//                    time = connectedNode.getCurrentTime();
//                    img.getHeader().setStamp(time);
//                    img.getHeader().setFrameId("android");
//                    img.getHeader().setSeq(sequenceNumber);
//                    img.setFormat("jpg");
//
//                    // make yuv compressed to jpeg
//                    yuvImage = new YuvImage(rawImageBuffer, ImageFormat.NV21, rawImageSize.width, rawImageSize.height, null);
//                    rect = new Rect(0, 0, rawImageSize.width, rawImageSize.height);
//                    Preconditions.checkState(yuvImage.compressToJpeg(rect, 20, stream));
//                    img.setData(stream.buffer().copy());
//                    stream.buffer().clear();
//
//                    // publish message
//                    publisher.publish(img);
//
//                    sequenceNumber++;
//                }
//            }
//        });
    }

    public void request(){
        final tms_ss_rcnn.obj_detectionRequest request = serviceClient.newMessage();


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
    }
}
