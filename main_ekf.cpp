#include <iostream>
#include <thread>

#include <pangolin/pangolin.h>
#include <Eigen/Core>

using namespace Eigen;
using std::cout;
using std::endl;

void DrawCamera(Eigen::Isometry3d &);

int main() {
    pangolin::CreateWindowAndBind("Camera Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam( //摆放一个相机
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(-1, 0, 1, 0, 0, 0, 1.0, 0.0, 0.0));
    pangolin::View &d_cam = pangolin::CreateDisplay() //创建一个窗口
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //消除颜色缓冲
        d_cam.Activate(s_cam);

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glColor3f(0.5, 0.5, 0.5);
        glLineWidth(0.5);
        pangolin::glDraw_z0(0.5f, 10);

        Eigen::AngleAxisd axis1(M_PI * 0.75, Eigen::Vector3d(0, 1, 0));
        Eigen::Quaterniond q1(axis1);
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.rotate(q1.toRotationMatrix());
        pose.pretranslate(Eigen::Vector3d(0, 0, 0.3));
        DrawCamera(pose);

        Eigen::AngleAxisd axis2(M_PI / 4, Eigen::Vector3d(0, 0, 1));
        Eigen::Quaterniond q2(axis2);
        Eigen::Quaterniond q3 = (q2 * q1).normalized();
        Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
        pose1.rotate(q3.toRotationMatrix());
        pose1.pretranslate(Eigen::Vector3d(0, 0.2, 0.3));
        DrawCamera(pose1);

        //画坐标原点
        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0.6, 0, 0);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0, 0.6, 0);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0, 0, 0);
        glVertex3d(0, 0, 0.6);
        glEnd();
        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}

void DrawCamera(Eigen::Isometry3d &pose) {
    glPushMatrix();
    Eigen::Matrix4f m = pose.matrix().cast<float>();
    glMultMatrixf((GLfloat *)m.data());

    const float w = 0.1;
    const float h = w * 0.75;
    const float z = w * 0.6;
    glColor3f(1, 0, 0);
    glLineWidth(2);
    glBegin(GL_LINES);
    //画相机模型
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);
    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);
    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);
    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);

    glEnd();
    glPopMatrix();
}
