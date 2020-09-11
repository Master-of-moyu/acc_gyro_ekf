#include <iostream>
#include <thread>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include "ekf_filter/sensor_fusion_ekf.h"

using namespace Eigen;
using std::cout;
using std::endl;

void DrawCamera(Eigen::Isometry3d &);

struct RawDataReader {
    FILE *fp;

    RawDataReader(const std::string file_name) {
        std::string temp = (file_name).c_str();
        fp = fopen((file_name).c_str(), "rb");
        if (fp == NULL) {
            fprintf(stderr, "%s fopen error!\n", file_name.c_str());
        }
        std::cout << "test file opened success." << std::endl;
    }

    ~RawDataReader() {
        if (fp) {
            fclose(fp);
            fp = NULL;
        }
    }

    template <typename T>
    void Read(T *data, int size, const int N = 1) {
        fread(data, size, N, fp);
    }
};
std::string data_file;
std::unique_ptr<cardboard::SensorFusionEkf> ekf_tracker = nullptr;

int main(int argc, const char *argv[]) {
    pangolin::CreateWindowAndBind("Camera Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam( //摆放一个相机
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -2, 1, 0, 0, 0, pangolin::AxisY));

    pangolin::View &d_cam = pangolin::CreateDisplay() //创建一个窗口
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    data_file = argv[1];
    RawDataReader reader(data_file);
    Eigen::Quaterniond camera_to_body_rotation =
        Eigen::Quaterniond((Eigen::Matrix3d() << 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0).finished());
    ekf_tracker = std::make_unique<cardboard::SensorFusionEkf>();
    ekf_tracker->SetBiasEstimationEnabled(true);

    unsigned char type;
    double img_time;
    int width, height;

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //消除颜色缓冲
        d_cam.Activate(s_cam);

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glColor3f(0.5, 0.5, 0.5);
        glLineWidth(0.5);
        pangolin::glDraw_z0(0.5f, 10);

        reader.Read<unsigned char>(&type, sizeof(unsigned char));
        if (type == 0) {
            reader.Read<double>(&img_time, sizeof(double));
            reader.Read<int>(&width, sizeof(int));
            reader.Read<int>(&height, sizeof(int));
            unsigned char *data = new unsigned char[width * height];
            reader.Read<unsigned char>(data, sizeof(unsigned char), width * height);

            cardboard::PoseState pose_state = ekf_tracker->GetLatestPoseState();
            cardboard::Rotation predicted_rotation = pose_state.sensor_from_start_rotation;

            Eigen::Quaterniond q_temp;
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

            q_temp.x() = static_cast<float>(predicted_rotation.GetQuaternion()[0]);
            q_temp.y() = static_cast<float>(predicted_rotation.GetQuaternion()[1]);
            q_temp.z() = static_cast<float>(predicted_rotation.GetQuaternion()[2]);
            q_temp.w() = static_cast<float>(predicted_rotation.GetQuaternion()[3]);
            Eigen::Quaterniond qs = q_temp.conjugate() * camera_to_body_rotation;
            pose.rotate(qs.toRotationMatrix());
            pose.pretranslate(Eigen::Vector3d(0, 0, 0.2));
            DrawCamera(pose);

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
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        } else if (type == 0x02) { // acc
            double acc_time;
            double acc[3];
            reader.Read<double>(&acc_time, sizeof(double));
            reader.Read<double>(acc, sizeof(double), 3);

            cardboard::AccelerometerData acc_data;
            acc_data.sensor_timestamp_ns = acc_time * 1e9;
            acc_data.data.Set(acc[0], acc[1], acc[2]);
            acc_data.system_timestamp = acc_data.sensor_timestamp_ns;
            ekf_tracker->ProcessAccelerometerSample(acc_data);
        } else if (type == 0x01) { // gyro
            double gyro_time;
            double gyro[3];
            reader.Read<double>(&gyro_time, sizeof(double));
            reader.Read<double>(gyro, sizeof(double), 3);

            cardboard::GyroscopeData gyr_data;
            gyr_data.sensor_timestamp_ns = gyro_time * 1e9;
            gyr_data.data.Set(gyro[0], gyro[1], gyro[2]);
            gyr_data.system_timestamp = gyr_data.sensor_timestamp_ns;
            ekf_tracker->ProcessGyroscopeSample(gyr_data);
        } else if (type == 17) { // attitude
            double att_time;
            double atts[4];
            reader.Read<double>(&att_time, sizeof(double));
            reader.Read<double>(atts, sizeof(double), 4);
        } else if (type == 18) { // gravity
            double grav_time;
            double grav[3];
            reader.Read<double>(&grav_time, sizeof(double));
            reader.Read<double>(grav, sizeof(double), 3);
        } else {
            break;
        }
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
