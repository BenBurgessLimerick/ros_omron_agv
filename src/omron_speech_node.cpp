// ROS OMRON driver
//------------------

#include <ArNetworking/ArClientRatioDrive.h>
#include <ArNetworking/ArNetworking.h>
#include <Aria/Aria.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

class OmronSpeech
{
public:
    OmronSpeech(ArClientBase *client, ros::NodeHandle *nh, std::string topic = "/omron/say");
    void SayCallback(const std_msgs::String::ConstPtr &msg);
    void SetVolCallback(const std_msgs::Int32::ConstPtr &msg);
    void audio_vol_cb(ArNetPacket *packet);

protected:
    ArClientBase *myClient;
    ros::NodeHandle *_nh;
    ros::Subscriber saySub;
    ros::Subscriber volSub;
    ArFunctor1C<OmronSpeech, ArNetPacket *> arAudioVolCB;
};

OmronSpeech::OmronSpeech(ArClientBase *client, ros::NodeHandle *nh, std::string topic)
    : myClient(client),
      _nh(nh),
      arAudioVolCB(this, &OmronSpeech::audio_vol_cb)
{
    saySub = _nh->subscribe(topic, 10,
                            &OmronSpeech::SayCallback, this);

    volSub = _nh->subscribe("/omron/set_volume", 10,
                            &OmronSpeech::SetVolCallback, this);

    // myClient->addHandler("AudioVol", &arAudioVolCB);
    // myClient->request("AudioVol",
    //                   1000);

    // for (int i = 700; i < 800; i++)
    // {
    //     auto out = myClient->getName(i);
    //     ROS_INFO("ID: %d = %s", i, out);
    // }
}

void OmronSpeech::audio_vol_cb(ArNetPacket *packet)
{

    int out = (packet->bufToUByte4());
    ROS_INFO("Audio vol: %d", out);
}

void OmronSpeech::SetVolCallback(const std_msgs::Int32::ConstPtr &msg)
{
    ROS_INFO("Setting Omron volume to: %d", msg->data);
    ArNetPacket p;
    p.byte4ToBuf(msg->data);
    myClient->requestOnce("setAudioVol", &p);
}

void OmronSpeech::SayCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Omron saying: %s", msg->data.c_str());
    ArNetPacket p;
    p.strToBuf(msg->data.c_str());
    myClient->requestOnce("say", &p);
}

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "omron_speech");

    // make node handle
    ros::NodeHandle n, np("~");

    // Aria
    Aria::init();

    // Create our client object.
    ArClientBase client;

    // Set the magical protocol
    client.enforceProtocolVersion("5MTX");

    // Settings
    ArArgumentBuilder args;
    //--------
    // HOST
    args.addPlain("-host");
    std::string sparam;
    if (np.getParam("host", sparam))
    {
        args.addPlain(sparam.c_str());
    }
    else
    {
        args.addPlain("172.19.21.203"); // Default IP
    }

    // PORT
    args.addPlain("-p");
    if (np.getParam("port", sparam))
    {
        args.addPlain(sparam.c_str());
    }
    else
    {
        args.addPlain("7272"); // Default PORT
    }

    // USER
    args.addPlain("-u");
    if (np.getParam("user", sparam))
    {
        args.addPlain(sparam.c_str());
    }
    else
    {
        args.addPlain("steve"); // Default user
    }
    // NO PASSWD
    args.addPlain("-np");

    ArClientSimpleConnector clientConnector(&args);

    // Reard in args
    clientConnector.parseArgs();

    // Connect
    if (!clientConnector.connectClient(&client))
    {
        if (client.wasRejected())
            ROS_ERROR("Server '%s' rejected connection, exiting\n",
                      client.getHost());
        else
            ROS_ERROR("Could not connect to server '%s', exiting\n",
                      client.getHost());
        exit(1);
    }

    ROS_INFO("Omron Speech, connected to server.\n");

    // Setup the status pub object
    OmronSpeech speechNode(&client, &n);

    client.runAsync();

    ros::spin();

    client.disconnect();
    Aria::exit(0);
}
