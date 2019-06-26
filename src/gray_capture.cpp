#include <iostream>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include <iomanip>
#include <fstream>
#include <string>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>

//#include <opencv2/opencv.hpp>  // checked at opencv 3.4.1
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/structured_light.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

#define WINDOWWIDTH 1920
#define WINDOWHEIGHT 1080
//#define SHIFT_WINDOWWIDTH 1920
//#define SHIFT_WINDOWHEIGHT 1080
#define SHIFT_WINDOWWIDTH 1280
#define SHIFT_WINDOWHEIGHT 800
#define GRAYCODEWIDTHSTEP 5
#define GRAYCODEHEIGHTSTEP 5
#define GRAYCODEWIDTH WINDOWWIDTH / GRAYCODEWIDTHSTEP
#define GRAYCODEHEIGHT WINDOWHEIGHT / GRAYCODEHEIGHTSTEP
#define WHITETHRESHOLD 5
#define BLACKTHRESHOLD 40



void terminateCamera() {}

// Camera to Projector
struct C2P {
  int cx;
  int cy;
  int px;
  int py;
  C2P(int camera_x, int camera_y, int proj_x, int proj_y) {
    cx = camera_x;
    cy = camera_y;
    px = proj_x;
    py = proj_y;
  }
};

int main(int argc, char **argv) {
  ROS_INFO("[gray_capture***START***");

  //ノード名の初期化
  ros::init(argc, argv, "gray_image_publisher");

  //ROSシステムと通信のためのノードのハンドルを宣言
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("gray_image",1);

  ros::Publisher cmd_pub = n.advertise<std_msgs::Bool>("g_command",1);
  ros::Rate loop_rate(60);

  // -----------------------------------
  // ----- Prepare graycode images -----
  // -----------------------------------
  cv::structured_light::GrayCodePattern::Params params;
  params.width = GRAYCODEWIDTH;
  params.height = GRAYCODEHEIGHT;

  std::string cmd1("v4l2-ctl -d /dev/video1 -c brightness=40");
  std::string cmd2("v4l2-ctl -d /dev/video1 -c exposure_auto=1");
  std::string cmd3("v4l2-ctl -d /dev/video1 -c exposure_auto_priority=0");
  std::string cmd4("v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=0");
  std::string cmd5("v4l2-ctl -d /dev/video1 -c exposure_absolute=20");
  std::string cmd6("v4l2-ctl -d /dev/video1 -c focus_auto=0");

  system(cmd6.c_str());

  std::vector<cv::Mat> graycodes;
  std::cout<<"read codes"<<std::endl;

  std::string gname;
  int pattern_max = 40;
  for(int i=1;i<=pattern_max;i++){
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << i;
    gname = "/home/mizobuchi/ros_workspace/src/phaseshift/gray_pattern/pattern_" + oss.str() + ".png";
    cv::Mat g_code = cv::imread(gname.c_str());
    graycodes.push_back(g_code);
  }


  // -----------------------------
  // ----- Prepare cv window -----
  // -----------------------------
  std::cout<<"name window"<<std::endl;
  cv::namedWindow("Pattern", cv::WINDOW_NORMAL);
  cv::resizeWindow("Pattern", GRAYCODEWIDTH, GRAYCODEHEIGHT);
  //cv::namedWindow("win", cv::WINDOW_NORMAL);
  // 2枚目のディスプレイにフルスクリーン表示
  std::cout<<"show window"<<std::endl;
  cv::moveWindow("Pattern", 1920, 0);
  //cv::moveWindow("Pattern", 1280, 0);
  cv::setWindowProperty("Pattern", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

  std::cout<<"show pattern"<<std::endl;
  cv::imshow("Pattern", graycodes.at(0));
  std::cout<<"camera capture"<<std::endl;

  //----------------------------------------------------

  cv::VideoCapture cap(1);//デバイスのオープン
    //cap.open(0);//こっちでも良い．

  if(!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
    {
        //読み込みに失敗したときの処理
        return -1;
    }
  cap.set(cv::CAP_PROP_FPS,5);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,960);
  cv::Mat frame; //取得したフレーム
  int cnt = 0;
  int check = 0;
  std::ofstream ofs("/home/mizobuchi/ros_workspace/src/phaseshift/gray_picture_List.txt");
  std::string fname;

  int k=1;
  bool mode = false;
  int p_i=0;
  while(ros::ok()){
      cap.read(frame);
      //std::cout<<frame.channels() << std::endl;
      cv::imshow("win", frame);//画像を表示．

      if(mode){
        //std::cout<<"capture "<<p_i<<std::endl;
        std::ostringstream oss;
        oss << std::setfill('0') << std::setw(2) << p_i;
        fname = "/home/mizobuchi/ros_workspace/src/phaseshift/gray_capture/cam_" + oss.str() + ".png";
        ofs<<fname.c_str()<<std::endl;
        cv::imwrite(fname.c_str(), frame);
        p_i++;
        if(p_i==pattern_max){
          std_msgs::Bool msg;
          msg.data = true;
          cmd_pub.publish(msg);
          std::cout<<"Finished and published message!"<<std::endl;
          mode=false;
          p_i=0;
        }
      }
      if(mode){
        cv::imshow("Pattern", graycodes.at(p_i));
        //std::cout<<"projection pattern "<<p_i<<std::endl;
        //cv::waitKey(40);
        //p_i++;
        //if(p_i==pattern_max){
          //p_i=0;
        //}
      }



      const int key = cv::waitKey(1);
      if(key == 'q'/*113*/){
        std::cout<<"publish message!"<<std::endl;
          std_msgs::Bool msg;
          msg.data = true;
          cmd_pub.publish(msg);
      }
      else if(key == 's'/*115*/){
          //フレーム画像を保存する．
          //パターンの枚数だけループ処理をして画像を保存する.
          mode=true;
          std::cout<<"mode change!"<<std::endl;

	         /*for(int i = 0 ; i < pattern_max ; i++){
             cv::imshow("Pattern", graycodes.at(i));
             sleep(1);
             cap.read(frame);
             cv::imshow("win", frame);//画像を表示．
	           std::ostringstream oss;
	           //oss << std::setfill('0') << std::setw(2) << cnt++;
             oss << std::setfill('0') << std::setw(2) << i;
             fname = "/home/mizobuchi/ros_workspace/src/phaseshift/gray_capture/cam_" + oss.str() + ".png";
             ofs<<fname.c_str()<<std::endl;
             cv::imwrite(fname.c_str(), frame);
             //画像が切り替わってカメラがピントを合わせられる時間だけ処理を止める.
             cv::waitKey(400);
             */
             /*if(k>=pattern_max){
                std::cout<<"Pattern finished! publish message!"<<std::endl;
                k=0;
                cnt=0;
                std_msgs::Bool msg;
            	  msg.data = true;
                cmd_pub.publish(msg);
            }
            cv::imshow("Pattern", graycodes.at(k));
            k++;*/
          //}
          //std_msgs::Bool msg;
          //msg.data = true;
          //cmd_pub.publish(msg);
          //std::cout<<"Finished and published message!"<<std::endl;
          //cv::imwrite("img.png", frame);
      }




      ros::spinOnce();
      cap.read(frame);
      //std::cout<<frame.channels() << std::endl;
      cv::imshow("win", frame);//画像を表示．

      loop_rate.sleep();


  }
  cv::destroyAllWindows();

  //------------------------------

  //for (int i=0;i<10;i++) {
    // ディスプレイに表示->カメラバッファに反映されるまで待つ
    // 必要な待ち時間は使うカメラに依存
    //cv::waitKey(400);
  //}



  cv::waitKey(0);

}
