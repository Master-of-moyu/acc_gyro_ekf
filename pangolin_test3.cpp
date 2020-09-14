#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

int main(int argc, char** argv) {
    // 创建视窗
    pangolin::CreateWindowAndBind("MultiImage", 800, 600);
    // 启动深度测试
    glEnable(GL_DEPTH_TEST);
    // 设置摄像机
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(800, 600, 500, 500, 400, 300, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -4, 2, 0, 0, 0, pangolin::AxisY));
    // ---------- 创建三个视图 ---------- //
    pangolin::View& d_cam = pangolin::Display("cam")
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -800.0 / 600)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::View& cv_img_1 = pangolin::Display("image_1")
                                   .SetBounds(2.0 / 3.0f, 1.0f, 0.0, 1.0 / 3.0f, 640.0 / 480);
    //    .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::View& cv_img_2 = pangolin::Display("image_2")
                                   .SetBounds(1.0 / 3.0, 2.0 / 3.0f, 0.0f, 1.0 / 3.0, 640.0 / 480);
    //    .SetLock(pangolin::LockRight, pangolin::LockBottom);

    pangolin::View& cv_img_3 = pangolin::Display("image_3")
                                   .SetBounds(0.0 / 3.0, 1.0 / 3.0f, 0.0f, 1.0 / 3.0, 752.0 / 480);
    //    .SetLock(pangolin::LockRight, pangolin::LockBottom);
    // 创建glTexture容器用于读取图像
    pangolin::GlTexture imgTexture1(640, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
    pangolin::GlTexture imgTexture2(640, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);
    pangolin::GlTexture imgTexture3(752, 480, GL_RGB, false, 0, GL_BGR, GL_UNSIGNED_BYTE);

    while (!pangolin::ShouldQuit()) {
        // 清空颜色和深度缓存
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // 启动相机
        d_cam.Activate(s_cam);

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glColor3f(0.5, 0.5, 0.5);
        glLineWidth(0.5);
        pangolin::glDraw_z0(0.5f, 10);
        {
            // 添加一个立方体
            glColor3f(1.0f, 1.0f, 1.0f);
            pangolin::glDrawColouredCube();

            // 绘制坐标系
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
        // 从文件读取图像
        cv::Mat img1 = cv::imread("./image/1.png");
        cv::Mat img2 = cv::imread("./image/2.png");
        cv::Mat img3 = cv::imread("./image/3.png");
        // 向GPU装载图像
        imgTexture1.Upload(img1.data, GL_BGR, GL_UNSIGNED_BYTE);
        imgTexture2.Upload(img2.data, GL_BGR, GL_UNSIGNED_BYTE);
        imgTexture3.Upload(img3.data, GL_BGR, GL_UNSIGNED_BYTE);
        // 显示图像
        cv_img_1.Activate();
        glColor3f(1.0f, 1.0f, 1.0f);         // 设置默认背景色，对于显示图片来说，不设置也没关系
        imgTexture1.RenderToViewportFlipY(); // 需要反转Y轴，否则输出是倒着的

        cv_img_2.Activate();
        glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
        imgTexture2.RenderToViewportFlipY();

        cv_img_3.Activate();
        glColor3f(1.0f, 1.0f, 1.0f); // 设置默认背景色，对于显示图片来说，不设置也没关系
        imgTexture3.RenderToViewportFlipY();

        pangolin::FinishFrame();
    }

    return 0;
}
