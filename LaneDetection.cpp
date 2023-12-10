// 소스 코드
// #define ROS              // ROS 매크로 (만약 ROS에서 진행하려면 주석 해제)

#define CAMERA_SHOW
#define CAMERA_SHOW_MORE

#ifdef ROS  // ros가 정의되어 있다면 아래 실행
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sys/wait.h>
#include <termio.h>
#include <unistd.h>
#include "std_msgs/int16.h"

#endif

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>

#define PI 3.1415926

using namespace cv;
using namespace std;

#define HIGH 1
#define LOW 0


const int Width = 320;
const int Height = 240;
const int XHalf = (Width / 2);
const int YHalf = (Height / 2);
const int YPoint[2] = { 20, YHalf - 20 };
const float slope_threshold = 0.5;
const Scalar Red = Scalar(0, 0, 255);
const Scalar Blue = Scalar(255, 0, 0);
const Scalar Yellow = Scalar(50, 250, 250);
const Scalar Sky = Scalar(215, 200, 60);
const Scalar Pink = Scalar(220, 110, 230);


void show_lines(Mat& img, vector<Vec4i>& lines, Scalar color = Scalar(0, 0, 0), int thickness = 2) {
    bool color_gen = false;
    if (color == Scalar(0, 0, 0))
        color_gen = true;
    for (int i = 0; i < lines.size(); i++)
    {
        if (color_gen == true)
            color = Scalar(rand() % 256, rand() % 256, rand() % 256);
        line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), color, thickness);
    }
}

// lines 는 허프변환의 리턴값
// left_lines와 right_lines에 왼쪽, 오른쪽 차선 후보들을 계산해 저장함
void split_left_right(vector<Vec4i> lines, vector<Vec4i>& left_lines, vector<Vec4i>& right_lines)
{
    vector<float> slopes;
    vector<Vec4i> new_lines;
    //compute slopes (dy/dx) for each line and exclude the lines with low slope
    for (int i = 0; i < lines.size(); i++)
    {
        // lines에 저장된 2개 점의 정보를 얻어냄
        int x1 = lines[i][0];
        int y1 = lines[i][1];
        int x2 = lines[i][2];
        int y2 = lines[i][3];

        float slope;
        // 2개 점의 기울기 계산

        if (x2 - x1 == 0) {  // 수직선은 기울기가 무한대
            slope = 999.0;
        }
        else { // 기울기 계산 
            slope = (y2 - y1) / float(x2 - x1);
        }
        //Filter lines based on slope 
        // 기울기와 기울기의 임계값과 비교해 큰것만 차선으로 택함
        // 기울기 임계값은 45도임
        // 결과적으로 new_lines 에 차선 후보들이 저장됨
        if (abs(slope) > slope_threshold) {
            slopes.push_back(slope);
            new_lines.push_back(lines[i]);
        }
    }
	for (int i = 0; i < new_lines.size(); i++) {
		Vec4i line = new_lines[i];
		float slope = slopes[i];

		int x1 = line[0];
		int y1 = line[1];
		int x2 = line[2];
		int y2 = line[3];

		// 기울기가 양수이고
		// x1 x2 좌표가 모두 오른쪽에있으면 오른쪽 차선으로 분류
		if (slope > 0 && x1 > XHalf && x2 > XHalf)// both x1 and 2 are on the right side and slope>0 
			right_lines.push_back(line);
		// 기울기가 음수이고
		// x1 x2 좌표가 모두 왼쪽에있으면 왼쪽 차선으로 분류
		else if (slope < 0 && x1 < XHalf && x2 < XHalf)//slope < 0 && x1 < cx && x2 < cx
			left_lines.push_back(line);
	}
}


// 직선의 기울기와 절편을 구하는 함수
// find_line_params()는 직선에서 기울기(m)와 절편(b)을 구하는 함수
bool find_line_params(vector<Vec4i>& left_lines, float* left_m, float* left_b)
{
    float left_avg_x = 0, left_avg_y = 0, left_avg_slope = 0;
    if (left_lines.size() == 0)
        return false;
   
    for (int i = 0; i < left_lines.size(); i++)//calculate right avg of x and y and slope
    {
        //line(roi, Point (left_lines [i][0], left_lines [i][1]), Point (left_lines [i] [2], left_lines[i] [3]), color, 3): 
        left_avg_x += left_lines[i][0];
        left_avg_x += left_lines[i][2];
        left_avg_y += left_lines[i][1];
        left_avg_y += left_lines[i][3];
        left_avg_slope += (left_lines[i][3] - left_lines[i][1]) / (float)(left_lines[i][2] - left_lines[i][0]);
    }

    // x좌표들의 평균, y좌표들의 평균, 기울기들의 평균을 구함     
	left_avg_x = left_avg_x / (left_lines.size() * 2);
	left_avg_y = left_avg_y / (left_lines.size() * 2); 
    left_avg_slope =  left_avg_slope / left_lines.size();

	// return values
    *left_m = left_avg_slope;
	//b=y-mx //find Y intercet
	*left_b = left_avg_y - left_avg_slope * left_avg_x;
	return true;
}


// find_lines 함수
// 왼쪽, 오른쪽 차선(left_lines, right_lines)를 이용해
// 직선의 방정식을 구함
void find_lines(Mat& img, vector<cv::Vec4i>& left_lines, vector<cv::Vec4i>& right_lines, float* lane_center, Point& vanishing_pt, float* leftX_inter, float* rightX_inter) {
    
    static float left_slope_mem = -1, right_slope_mem = 1, right_b_mem = 0, left_b_mem = 0;
    float left_b = 0, right_b = 0, left_m = 0, right_m = 0;

    // find_line_params()는 직선에서 기울기(m)와 절편(b)을 구하는 함수
    // 왼쪽 차선에 대해 수행
    bool draw_left = find_line_params(left_lines, &left_m, &left_b);

    // true 라면 실행
    if (draw_left) { // y = mx + b => x0 = (y0-b)/m
        float left_x0 = (-left_b) / left_m;
        float left_x120 = (YHalf - left_b) / left_m;
        left_slope_mem = left_m;
        left_b_mem = left_b;
#ifdef CAMERA_SHOW
        // 왼쪽 차선을 파란색으로 그린다.
        line(img, Point(left_x0, 0), Point(left_x120, YHalf), Blue, 3);
        cout << left_lines.size() << " left lines.";
        *leftX_inter = left_x120;
#endif 
    }
    // false라면 no left line
    else {
        cout << "#tNo Left Line,";
    }
    
    // 오른쪽 차선에 대해 수행
    bool draw_right = find_line_params(right_lines, &right_m, &right_b);
    if (draw_right) { // y = mx + b => x0 = (y0-b)/m
        float right_x0 = (-right_b) / right_m;
        float right_x120 = (YHalf - right_b) / right_m;
        right_slope_mem = right_m;
        right_b_mem = right_b;
#ifdef CAMERA_SHOW
        // 오른쪽 차선을 빨간색으로 그린다.
        line(img, Point(right_x0, 0), Point(right_x120, YHalf), Red, 3);
        *rightX_inter = right_x120;
#endif 
        cout << right_lines.size() << " righ lines." << endl;
    }
    else {
        cout << "#tNo Right Line," << endl;
    }

    // y = mx + b ==> x0 = (y0-b)/m
    // 위 식을 이용해 x 좌표를 구함
    float left_xPt[2] = {0.0f, 0.0f};
    float right_xPt[2] = {0.0f, 0.0f};

    left_xPt[LOW] = ((YPoint[LOW] - left_b_mem) / left_slope_mem); 
    right_xPt[LOW] = ((YPoint[LOW] - right_b_mem) / right_slope_mem); 
    // Idistance [LOW] = XHalf left_xPt [LOW]:
    // rdistance [LOW] = right_Pt [LOW] - XHalf;
    
    left_xPt[HIGH] = ((YPoint[HIGH] - left_b_mem) / left_slope_mem); 
    right_xPt[HIGH] = ((YPoint[HIGH] - right_b_mem) / right_slope_mem);
    // Idistance [HIGH] = XHalf left_xPt [HIGH);
    // rdistance [HIGH] = right xPt[HIGH - XHalf;
        
    lane_center[LOW] = (left_xPt[LOW] + right_xPt[LOW]) / 2.0;
    lane_center[HIGH] = (left_xPt[HIGH] + right_xPt[HIGH]) / 2.0;
    
    float Vx = -(left_b_mem - right_b_mem) / (left_slope_mem - right_slope_mem);
    float Vy = left_slope_mem * Vx + left_b_mem;
    vanishing_pt = Point(Vx, Vy);
}

// img_process() 함수
int img_process(Mat& frame)
{
    static float leftX = 0, rightX = 0;
    Mat grayframe, edge_frame, roi_gray_ch3;
    Mat roi;
    // cvtColor(frame, grayframe, COLOR_BGR2GRAY);  // 그레이 스케일 변환
    Rect rect_roi(0, YHalf, Width, YHalf); // roi 설정
    roi = frame(rect_roi); // roi 


    cvtColor(roi, grayframe, COLOR_BGR2GRAY); // roi 부분을 그레이 스케일로
    GaussianBlur(grayframe, grayframe, Size(3, 3), 1.5);  // 가우시안 블러 처리
    cvtColor(grayframe, roi_gray_ch3, COLOR_GRAY2BGR);   // 그레이를 bgr로 변환(결과는 색은 그레이인데 채널은 3개임)
    Canny(grayframe, edge_frame, 70, 150, 3); // canny 엣지 추출

    vector<cv::Vec4i> lines_set;

    cv::HoughLinesP(edge_frame, lines_set, 1, PI / 180, 30, 30, 10); // 허프 변환 (lines_set 에 저장)
#ifdef CAMERA_SHOW_MORE
    show_lines(roi_gray_ch3, lines_set); // show_lines()는 허브 변환으로 찾은 직선을 화면에 그려주는 함수
    // 찾은 직선을 모두 그려 ros_gray_ch3에 저장)
#endif
    vector<Vec4i> right_lines;
    vector<Vec4i> left_lines;
    split_left_right(lines_set, left_lines, right_lines); // 차선을 왼쪽, 오른쪽으로 분리
#ifdef CAMERA_SHOW
    // 아래 2개는 차선 후보만 추출해 그림 (왼쪽 하늘색, 오른쪽 핑크)
    show_lines(roi, left_lines, Sky, 2);
    show_lines(roi, right_lines, Pink, 2);
#endif
    float lane_center[2] = {0.0, 0.0}; // Idistance [2], rdistance [2]:
    Point vpt;

    // lane_center, vpt, leftX, rightX 가 리턴값
    // lane_center 두 차선의 가운데점(2개)  - lane_center[2];
    // vpt는 소실점 ( 차선의 연장선의 만나는 점)
    find_lines(roi, left_lines, right_lines, lane_center, vpt, &leftX, &rightX);

    // (rightX - vpt.x) 는 오른쪽 차선 - 소실점의x 좌표 = a
    // (vpt.x - leftX) 는 소실점의 x좌표 - 왼쪽 차선  = b
    // a/b 계산
    // 만약 1.1 보다 더 크면 차선이 오른쪽으로 향함
    // 0.9보다 작으면 왼쪽으로 향함
    // 그 외에는 차선이 직선으로 향함
    float ratio = (rightX - vpt.x) / (vpt.x - leftX);

#ifdef CAMERA_SHOW
    if (ratio > 1.1) // 1.1 보다 클 경우 오른쪽으로 선 그림
        arrowedLine(frame, Point(XHalf, YHalf), Point(XHalf - ratio * 10, YHalf), Scalar(255, 120, 40), 4);
    else if (ratio < 0.9) // 0.9 보다 작을 경우 왼쪽으로 선 그림
        arrowedLine(frame, Point(XHalf, YHalf), Point(XHalf + 1.0 / ratio * 10, YHalf), Scalar(40, 120, 255), 4);
    else // 그 외는 직진 - 한가운데 원을 그림
        circle(frame, Point(vpt.x, YHalf - 3), 5, Scalar(255, 70, 255), -1);

    int differ[2]= {0.0f, 0.0f};
    for (int i = LOW; i <= HIGH; i++) {
        //differ [i]=rdistance [i] - Idistance [i];
        //circle(roi, Point (XHalf, YPoint [i]), 5, Scalar (250, 250, 250), -1);
        //circle(roi, Point (XHalf + differ [i], YPoint [i]), 5, Scalar (0, 0, 255), 2);
        //putText(roi, format("%3d%3d%3d", (int)rdistance [i], (int) Idistance[i], differ [i]), Point (XHalf - 100, YHalf/2 + i×30), FO.
        differ[i] = lane_center[i] - XHalf;
        circle(roi, Point(XHalf, YPoint[i]), 5, Scalar(250, 250, 250), -1);
        circle(roi, Point(lane_center[i], YPoint[i]), 5, Scalar(0, 0, 255), 2);
        //putText(roi, format("%3d - %3d = %3d", (int)lane_center[i], (int)XHalf, differ[i]), Point(XHalf-100, YHalf / 2 + i * 30), FONT_
        putText(roi, format("%3d - %3d = %3d", static_cast<int>(lane_center[i]), (int)XHalf, differ[i]), Point(XHalf - 100, YHalf / 2 + i * 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    }

    // 소실점 그리기
    circle(roi, Point(vpt.x, YHalf), 7, Scalar(255, 0, 120), 3);
    imshow("roi", roi);
    imshow("edgeframe", edge_frame);
#endif
#ifdef CAMERA_SHOW_MORE
    imshow("frame", frame);
    imshow("roi_gray_ch3", roi_gray_ch3);
#endif
    return ratio * 100;
}


// main 함수 
int main(int argc, char** argv) {
    #ifdef ROS                       // ROS가 정의되어 있다면 웹캠 객체 생성
    VideoCapture cap(0);
    #else                            // 아니라면 동영상 객체 생성
    VideoCapture cap("drive.mp4");
    #endif

    Mat frame;

    // 열리지 않았다면 오류 문구 출력
    if (!cap.isOpened()) {
        std::cout << "No camera or No file!" << std::endl;
        return -1;
    }
   
    #ifdef ROS   // ros가 정의되어 있다면 실행
    ros::init(argc, argv, "cam_msg_publisher"); // 노드 초기화
    ros::NodeHandle nh;
    std::msgs::Int16 cam_msg;  // 메시지 객체 생성
    ros::Publisher pub = nh.advertise<std_msgs::Int16>("cam_msg"); // 퍼블리셔 생성

    int init_past = 1;
    ros::Rate loop_rate(50);
    cout << "start" << endl;
    #endif  


    int differ, key, fr_no = 0;
    bool capture = true;

    //clock_t tStart = clock(); 
    for (;;) {
        if (capture) {
            cap >> frame;
            if (frame.empty())
                break;
        }
        if ((key = waitKey(30)) >= 0) {
            if (key == 27)
                break;
            else if (key == ' ') {
                capture = !capture;
            }
        }

        if (capture == false)
            continue;

        fr_no++;
        resize(frame, frame, Size(Width, Height));
        differ = img_process(frame);

#ifdef ROS
        cam_msg.data = differ;
        pub.publish(cam_msg);
        loop_rate, sleep();
#else
        std::cout << fr_no << ":" << differ << endl;
#endif
    }

    std::cout << "Camera off" << endl;
    return 0;
}
