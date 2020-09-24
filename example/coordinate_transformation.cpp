#include <pangolin/pangolin.h>
#include <stdio.h>
#include "../types.h"
#include "../mylog.h"

void draw_axis() {
    glLineWidth(3);
    glBegin(GL_LINES);
    glColor3f(0.8f, 0.f, 0.f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.5f, 0.0f, 0.0f);

    glColor3f(0.f, 0.8f, 0.f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.5f, 0.0f);

    glColor3f(0.2f, 0.2f, 1.f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.5f);
    glEnd();
}

void draw_camera(Eigen::Matrix4d& pose) {
    glPushMatrix();
    glMultMatrixd(pose.data());
    // 绘制相机轮廓线
    const float w = 0.2;
    const float h = w * 1.3;
    const float z = w * 0.6;

    glLineWidth(2);
    glBegin(GL_LINES);
    glColor3f(0.0f, 1.0f, 1.0f);
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

void draw_points(const Eigen::Vector3d& p) {
    glPointSize(4);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.95, 0.8);
    glVertex3f(p[0], p[1], p[2]);
    glEnd();
}

int main(int argc, char** argv) {
    LoggingSupport::set_log_level(4);
    // 初始化视窗
    pangolin::CreateWindowAndBind("Main", 800, 600);
    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam_(
        pangolin::ProjectionMatrix(800, 600, 500, 500, 400, 300, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -3, 2, 0, 0, 0, pangolin::AxisY));

    pangolin::View& d_cam_ = pangolin::CreateDisplay()
                                 .SetBounds(0.0, 1.0, 0.0, 1.0, -800.0f / 600.0f)
                                 .SetHandler(new pangolin::Handler3D(s_cam_));

    bool printed = false;

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam_.Activate(s_cam_);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glColor3f(0.5, 0.5, 0.5);
        glLineWidth(0.5);
        pangolin::glDraw_z0(0.5f, 10);

        Eigen::Matrix4d pose1;
        pose1.setIdentity();

        Eigen::AngleAxisd axis1(M_PI_2, vector<3>(0, 0, 1));
        Eigen::AngleAxisd axis2(M_PI_2, vector<3>(0, 1, 0));
        quaternion q1(axis1);
        quaternion q2(axis2);
        quaternion camera_q = (q2 * q1).normalized();
        vector<3> camera_p(0.5, 0, 0);
        vector<3> point_p(2, 0, 0);

        pose1.block<3, 3>(0, 0) = camera_q.toRotationMatrix();
        pose1.block<3, 1>(0, 3) = camera_p;

        Eigen::Matrix4d pose2;
        pose2.setIdentity();
        {
            Eigen::AngleAxisd axis1(M_PI_2 + 0.2, vector<3>(0, 0, 1));
            Eigen::AngleAxisd axis2(M_PI_2 + 0.3, vector<3>(0, 1, 0));
            quaternion q1(axis1);
            quaternion q2(axis2);
            quaternion camera_q = (q2 * q1).normalized();
            vector<3> camera_p(0.9, 0.8, 0);
            pose2.block<3, 3>(0, 0) = camera_q.toRotationMatrix();
            pose2.block<3, 1>(0, 3) = camera_p;
        }

        draw_camera(pose1);
        draw_camera(pose2);
        draw_axis();
        draw_points(point_p);

        if (!printed) {
            printed = true;
            quaternion camera_q1(pose1.block<3, 3>(0, 0));
            quaternion camera_q2(pose2.block<3, 3>(0, 0));
            vector<3> camera_p1 = pose1.block<3, 1>(0, 3);
            vector<3> camera_p2 = pose2.block<3, 1>(0, 3);

            // point projected into camera1 and camera2
            vector<3> proj_p1;
            vector<3> proj_p2;
            proj_p1 = camera_q1.conjugate() * (point_p - camera_p1);
            proj_p2 = camera_q2.conjugate() * (point_p - camera_p2);
            log_debug("point: %lf %lf %lf", point_p[0], point_p[1], point_p[2]);
            log_debug("proj_p1: %lf %lf %lf", proj_p1[0], proj_p1[1], proj_p1[2]);
            log_debug("proj_p2: %lf %lf %lf", proj_p2[0], proj_p2[1], proj_p2[2]);

            // pose from pose1 to pose2
            Eigen::Matrix4d T21;
            T21.setIdentity();
            T21 = pose2.inverse() * pose1;
            quaternion q21(T21.block<3, 3>(0, 0));
            vector<3> p21 = T21.block<3, 1>(0, 3);

            // point transform from camera1 to camera2
            vector<3> p_pose1_2_pose2;
            p_pose1_2_pose2 = q21 * proj_p1 + p21;
            log_debug("p_pose1_2_pose2: %lf %lf %lf", p_pose1_2_pose2[0], p_pose1_2_pose2[1], p_pose1_2_pose2[2]);
        }

        pangolin::FinishFrame();
    }

    return 0;
}
