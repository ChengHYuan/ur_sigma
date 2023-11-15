 //
// Created by jiy on 2021/11/2.
//

#include "ros/ros.h"
#include "dmp/dmp.h"
#include "dmp/DMPPointStamp.h"
#include <fstream>


#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

const int NUMBASES = 15;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmp_learn");
    ros::NodeHandle n;

    ifstream my_data("/home/jiyguo/ur_ws/src/dmp/data/dealdata22.txt",ios::in);
    if (!my_data.is_open())
    {
        cout << "Can not open this file" << endl;
        return -1;
    }

    ros::ServiceClient clientLfd = n.serviceClient<dmp::LearnDMPFromDemo>("learn_dmp_from_demo");

    dmp::LearnDMPFromDemo srv;
    std::vector<dmp::DMPPoint> points;
    std::vector<double> times;

    double t = 0;
    double dt = 0.01;
    double x,y,pre_x,pre_y;
    my_data>>x;
    my_data>>y;
    pre_x = x;
    pre_y = y;
    times.push_back(t);
    dmp::DMPPoint start;
    start.positions.push_back(x);
    start.velocities.push_back(0);
    start.positions.push_back(y);
    start.velocities.push_back(0);
    while(!my_data.eof())
    {
        t += dt;
        my_data>>x;
        my_data>>y;
        times.push_back(t);
        dmp::DMPPoint point;
        point.positions.push_back(x);
        point.velocities.push_back((x-pre_x)/dt);
        point.positions.push_back(y);
        point.velocities.push_back((y-pre_y)/dt);
        points.push_back(point);
        pre_x = x;
        pre_y = y;
    }

    double d_gain = 15;
    double k_gain = pow(d_gain,2) / 4.0;
//    double d_gain = k_gain/4;

    srv.request.k_gains = std::vector<double>{k_gain,k_gain};
    srv.request.d_gains = std::vector<double>{d_gain,d_gain};

    srv.request.num_bases = NUMBASES;
    srv.request.demo.points = points;
    srv.request.demo.times = times;

    XMLDocument doc;
    doc.LinkEndChild(doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\""));
    doc.LinkEndChild(doc.NewComment("this is a DMP library data"));
    XMLElement* root=doc.NewElement("DMPs");
    doc.InsertEndChild(root);

    string dim = to_string(srv.request.k_gains.size());
    root->SetAttribute("dim",dim.c_str());

    if(clientLfd.call(srv)){
        double tau = srv.response.tau;
        XMLElement* tau_element=doc.NewElement("tau");
        tau_element->LinkEndChild( doc.NewText( to_string(tau).c_str() ));
        root->LinkEndChild(tau_element);

        std::vector<dmp::DMPData>  dmpList = srv.response.dmp_list;
        for(int i=0;i<dmpList.size();++i){
            auto dmpData = dmpList[i];
            XMLElement* dmp_dataElement = doc.NewElement( "dmp" );
            string id = to_string(i);
            dmp_dataElement->SetAttribute("id",id.c_str());
            XMLElement* kElement = doc.NewElement( "kgain" );
            kElement->LinkEndChild( doc.NewText( to_string(dmpData.k_gain).c_str() ));
            XMLElement* dElement = doc.NewElement( "dgain" );
            dElement->LinkEndChild( doc.NewText( to_string(dmpData.d_gain).c_str() ));
            dmp_dataElement->SetAttribute("id",id.c_str());
            dmp_dataElement->LinkEndChild(kElement);
            dmp_dataElement->LinkEndChild(dElement);
            XMLElement* weightsElement = doc.NewElement( "weights" );
            weightsElement->SetAttribute("nums", to_string(dmpData.weights.size()).c_str());
            for (int j = 0;j<dmpData.weights.size();++j) {
                XMLElement* weightElement = doc.NewElement( "w" );
                weightElement->SetAttribute("id", to_string(j).c_str());
                weightElement->LinkEndChild( doc.NewText( to_string(dmpData.weights[j]).c_str() ));
                weightsElement->InsertEndChild( weightElement );
            }
            dmp_dataElement->LinkEndChild(weightsElement);
            root->LinkEndChild(dmp_dataElement);
        }
        XMLPrinter printer;
        doc.Print(&printer);
        cout<< printer.CStr() << endl;
        string id = "dmp3";
        //id.push_back(*argv[1]);
        id += ".xml";
        doc.SaveFile(id.c_str());
        ROS_INFO("DMP services call successful ! ");
    }else {
        ROS_INFO("Failled call DMP services! ");
    }
    return 0;
}