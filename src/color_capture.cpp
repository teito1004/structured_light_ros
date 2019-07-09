#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <sstream>
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
  ROS_INFO("[color_capture]***START***");

  //ノード名の初期化
  ros::init(argc, argv, "color_image_publisher");

  //ROSシステムと通信のためのノードのハンドルを宣言
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("color_image", 1);

  ros::Publisher cmd_pub = n.advertise<std_msgs::Bool>("command", 1);

  ros::Rate loop_rate(10);


  // -----------------------------------
  // ----- Prepare graycode images -----
  // -----------------------------------
  cv::structured_light::GrayCodePattern::Params params;
  params.width = GRAYCODEWIDTH;
  params.height = GRAYCODEHEIGHT;


  std::vector<cv::Mat> graycodes;
  std::cout<<"read codes"<<std::endl;


  cv::Mat w_code = cv::imread("/home/mizobuchi/ros_workspace/src/phaseshift/hamming_color_code/pattern_white.png");
  cv::Mat x_code = cv::imread("/home/mizobuchi/ros_workspace/src/phaseshift/hamming_color_code/pattern_x.png");
  cv::Mat y_code = cv::imread("/home/mizobuchi/ros_workspace/src/phaseshift/hamming_color_code/pattern_y.png");

  cv::Mat dm_code(x_code.size(),CV_8UC3,cv::Scalar::all(128));

  //graycodes.push_back(w_code);
  graycodes.push_back(dm_code);
  graycodes.push_back(x_code);
  graycodes.push_back(y_code);

  // -----------------------------
  // ----- Prepare cv window -----
  // -----------------------------
  std::cout<<"name window"<<std::endl;
  cv::namedWindow("Raw", cv::WINDOW_NORMAL);
  cv::namedWindow("Chess", cv::WINDOW_NORMAL);
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
	std::cout<<"device cant open : "<<std::endl;
        //読み込みに失敗したときの処理
        return -1;
    }
  cap.set(cv::CAP_PROP_FPS,20);
  cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,960);
  cv::Mat frame; //取得したフレーム
  cap.read(frame);
  cv::Mat gray(frame.size(), CV_8UC1);
  cv::Mat img(frame.size(), CV_8UC3);
  int cnt = 0;
  int check = 0;
  //std::ofstream ofs("picture_List.txt");
  std::string fname;
  std::string cmd1("v4l2-ctl -d /dev/video1 -c brightness=100");
  std::string cmd2("v4l2-ctl -d /dev/video1 -c exposure_auto=1");
  std::string cmd3("v4l2-ctl -d /dev/video1 -c exposure_auto_priority=0");
  std::string cmd4("v4l2-ctl -d /dev/video1 -c white_balance_temperature_auto=0");
  std::string cmd5("v4l2-ctl -d /dev/video1 -c exposure_absolute=30");
  std::string cmd6("v4l2-ctl -d /dev/video1 -c focus_auto=0");
  std::string cmd7("v4l2-ctl -d /dev/video1 -c gain=30");
  //std::string cmd("v4l2-ctl -d /dev/video1 -c brightness=255");
  system(cmd1.c_str());
  system(cmd3.c_str());
  system(cmd2.c_str());
  system(cmd4.c_str());
  system(cmd5.c_str());
  system(cmd6.c_str());
  system(cmd7.c_str());
  //system(cmd.c_str());
  //system(cmd.c_str());
  cv::Size patternsize(10,7);
  std::vector< cv::Point2f > corners;

  bool patternfound;
  cv::Mat r_img;
  while(ros::ok()){
      cap.read(frame);
      cv::resize(frame,r_img,cv::Size(),0.5,0.5);
      //std::cout<<frame.channels() << std::endl;
      cv::imshow("Raw", frame);//画像を表示．
      //gray = cvCreateMat(img->rows, img->cols, CV_8UC1);
      cv::cvtColor(r_img, gray, CV_BGR2GRAY);
      img = r_img.clone();
      patternfound = findChessboardCorners(gray, patternsize, corners);
      if(patternfound){
         drawChessboardCorners(img, patternsize, cv::Mat(corners), patternfound);
      }
      cv::imshow("Chess", img);//画像を表示．

      const int key = cv::waitKey(1);

      if(key == 'q'){
          std_msgs::Bool msg;
          msg.data = true;
          cmd_pub.publish(msg);

      }else if(key == 's'/*115*/){
          //フレーム画像を保存する．
          std::ostringstream oss;
          oss << std::setfill('0') << std::setw(2) << cnt++;
          fname = "/home/mizobuchi/ros_workspace/src/phaseshift/color_capture/cam_" + oss.str() + ".png";
	  std::cout<<"cam_"<<oss.str()<<std::endl;
          //ofs<<fname.c_str()<<std::endl;
          cv::imwrite(fname.c_str(), frame);
          check = cnt % 3;
          switch(check){
            case 0:
              std::cout<<"dm code"<<std::endl;
              cv::imshow("Pattern", graycodes.at(0));
              break;
            case 1:
              std::cout<<"x code"<<std::endl;
              cv::imshow("Pattern", graycodes.at(1));
              break;
            case 2:
              std::cout<<"y code"<<std::endl;
              cv::imshow("Pattern", graycodes.at(2));
              break;
          }
          cv::waitKey(400);
          //cv::imwrite("img.png", frame);
      }
      ros::spinOnce();
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
