#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp> 
#include <stdio.h>
#include <vector>
#include <sstream>
// #include <GL/glew.h>
// #include <GL/glfw3.h>
// #include <glm/glm.hpp>
#include <math.h>
// using namespace glm;
using namespace cv;
using namespace std;

void onMouse(int evt, int x, int y, int flags, void* param){
    bool stop = false;
    if(evt == EVENT_LBUTTONDOWN) {
        vector<Point>* ptPtr = (vector<Point>*)param;
        ptPtr->push_back(Point(x,y));
    }
    // if(evt == EVENT_RBUTTONDOWN){
    //     stop = true;
    // }
}

static Mat cannyThreshold(Mat imGray, Mat thresh, int lowThreshold, int ratio)
{
    blur( imGray, thresh, Size(3,3) );
    Canny( thresh, thresh, lowThreshold, lowThreshold*ratio, 3);
    return thresh;
    // dst = Scalar::all(0);
    // src.copyTo( dst, detected_edges);
    // imshow( window_name, dst );
}

bool appartient(vector<vector<Point>> list, Point elem){
    bool app = false;
    for(int i=0; i<list.size();i++){
        for(int j=0; j<list[i].size();j++){
            if(abs(elem.x-list[i][j].x)+abs(elem.y-list[i][j].y)<8){
                app = true;
            }
        }
        // auto res = find(begin(list[i]),end(list[i]),elem);
        // app = (res != end(list[i]));
    }
    return app;
}

int main(){
    Mat I = imread("../pot.jpg",1);
    if (I.empty()) {
        cout << "Image File " << "Not Found" << endl;
        // wait for any key press
        cin.get();
        return -1;
    }
    int w=I.size().width; int h=I.size().height;
    vector<Point> points;
    namedWindow("output");
    setMouseCallback("output", onMouse, (void*)&points);
    imshow("output", I);

    Mat img_gray;
    cvtColor(I, img_gray, COLOR_BGR2GRAY);
    Mat thresh;
    thresh = cannyThreshold(img_gray, thresh, 50, 3);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    namedWindow("contour");
    imshow("contour", thresh);

    int pt_ellipse = 20;
    vector<Point3f> pts;
    waitKey(0);
    Point center;
    Size axes;
    if (points.size()==3){
        cout<<points<<endl;
        center = Point((points[0].x+points[1].x)/2,points[0].y);
        axes = Size((points[1].x-points[0].x)/2, points[2].y-points[0].y);
        for (int theta=0; theta < pt_ellipse; theta++){

            pts.push_back(Point3f(cos(theta*(2*M_PI/pt_ellipse)),0,sin(theta*(2*M_PI/pt_ellipse))));
        }
        ellipse(I, center, axes, 0, 0, 360, Scalar(0,255,255), 2, LINE_AA);
        // fillPoly(I,points,Scalar(255, 255, 255));
        imshow("output", I);
    }
    waitKey(0);

    float r = axes.width;
    int num_profile = 60;
    if (points.size()==4){
        Point A = Point(points[0].x,center.y);
        Point B = Point(points[1].x,center.y);
        for (int i=0; i<num_profile; i++){
            float gap = (points[2].y-points[3].y)/num_profile;
            center.y -= gap;
            float maxDiff = axes.width;
            for (int diff=0; diff<maxDiff; diff++){
                Point Ap = Point(points[0].x-diff, center.y);
                Point Am = Point(points[0].x+diff, center.y);
                Point Bp = Point(points[1].x-diff, center.y);
                Point Bm = Point(points[1].x+diff, center.y);
                if (appartient(contours, Ap)){
                    A = Ap;
                }else if(appartient(contours, Am)){
                    A = Am;
                }
                if (appartient(contours, Bp)){
                    B = Bp;
                }else if(appartient(contours, Bm)){
                    B = Bm;
                }
            }
            center.x = (B.x+A.x)/2;
            cout<<axes.width<<endl;
            axes = Size((B.x-A.x)/2, axes.height);
            for (int theta=0;theta < pt_ellipse;theta++){
                pts.push_back(Point3f((axes.width/r)*cos(theta*(2*M_PI/pt_ellipse)),center.y,(axes.width/r)*sin(theta*(2*M_PI/pt_ellipse))));
            }
            ellipse(I, center, axes, 0, 0, 360, Scalar(0,255,255), 2, LINE_AA);
            imshow("output", I);
        }
    }
    cout<<pts[50]<<endl;

    waitKey(0);

    // vector<Point3f> points3d;
    // points3d.push_back(Point3f(0,0,0));
    // points3d.push_back(Point3f(1,0,0));
    // points3d.push_back(Point3f(0,1,0));
    // Mat cam;
    // cam = initCameraMatrix2D(points3d,points,I.size());
    // cout<<cam<<endl;

    // AlphaColor col(0,0,255,70);
    // int x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x8,y8;
    // getMouse(x1,y1);
    // getMouse(x2,y2);
    // getMouse(x3,y3);
    // getMouse(x7,y7);

    // x4=x3+x2-x1;
    // y4=y3+y2-y1;
    // int dx=x7-x3;
    // int dy=y7-y3;
    // x5=x1+dx; y5=y1+dy; x6=x2+dx; y6=y2+dy; x8=x4+dx; y8=y4+dy;

    // int fx1[]={x1,x2,x4,x3},fy1[]={y1,y2,y4,y3};                            // polygon
    // fillPoly(fx1,fy1,4,col);                                             // inside
    // drawPoly(fx1,fy1,4,col);
    // int fx2[]={x1,x2,x6,x5},fy2[]={y1,y2,y6,y5};                            // polygon
    // fillPoly(fx2,fy2,4,col);                                             // inside
    // drawPoly(fx2,fy2,4,col);
    // int fx3[]={x2,x4,x8,x6},fy3[]={y2,y4,y8,y6};                            // polygon
    // fillPoly(fx3,fy3,4,col);                                             // inside
    // drawPoly(fx3,fy3,4,col);
    // int fx4[]={x3,x4,x8,x7},fy4[]={y3,y4,y8,y7};                            // polygon
    // fillPoly(fx4,fy4,4,col);                                             // inside
    // drawPoly(fx4,fy4,4,col);
    // int fx5[]={x3,x1,x5,x7},fy5[]={y3,y1,y5,y7};                            // polygon
    // fillPoly(fx5,fy5,4,col);                                             // inside
    // drawPoly(fx5,fy5,4,col);
    // int fx6[]={x5,x6,x8,x7},fy6[]={y5,y6,y8,y7};                            // polygon
    // fillPoly(fx6,fy6,4,col);                                             // inside
    // drawPoly(fx6,fy6,4,col);

    // endGraphics();
    waitKey(0);
    return 0;
}