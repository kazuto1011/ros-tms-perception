package com.github.kazuto1011.tms_ss_rcnn_client.object_scouter;

import android.os.Handler;
import android.util.Log;

import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.io.IOException;
import java.util.Collections;
import java.util.List;

import tms_msg_db.Tmsdb;
import tms_msg_db.TmsdbGetData;
import tms_msg_db.TmsdbGetDataRequest;
import tms_msg_db.TmsdbGetDataResponse;
import tms_msg_db.TmsdbStamped;

/*
A ROS Node that reads continiously informations from the ROS TMS Database.
 */

public class DbReaderClient extends AbstractNodeMain {
    private String TAG = "DbReaderClient";

    // DB Reader Client
    private ServiceClient<TmsdbGetDataRequest, TmsdbGetDataResponse> serviceClient;

    private ConnectedNode mConnectedNode;
    private Handler handler;

    static public List<Tmsdb> Tmsdbs;
    public DbReaderClient(Handler handler) {
        this.handler = handler;
    }

    static public List<Tmsdb> getTmsdbs(){
       return Tmsdbs;
    }

    static public void setTmsdbs(List<Tmsdb> input){
        Tmsdbs = input;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ObjectScouter/DbReaderClient");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);

        try {
            serviceClient = connectedNode.newServiceClient("tms_db_reader", TmsdbGetData._TYPE);
        } catch (ServiceNotFoundException e) {
            throw new RosRuntimeException(e);
        }

        mConnectedNode = connectedNode;

        mConnectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                final TmsdbGetDataRequest request = serviceClient.newMessage();
                request.getTmsdb().setId(7005);

                serviceClient.call(request, new ServiceResponseListener<TmsdbGetDataResponse>() {
                    @Override
                    public void onSuccess(TmsdbGetDataResponse response) {
                        Log.i(TAG, "Succeeded to call service");
                        Tmsdbs.add(response.getTmsdb().get(0));

                    }

                    @Override
                    public void onFailure(RemoteException e) {
                        Log.i(TAG, "Failed to call service");
                        throw new RosRuntimeException(e);
                    }
                });

            }
        });


    }
}

