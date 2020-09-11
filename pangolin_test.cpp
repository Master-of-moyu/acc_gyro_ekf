#include <pangolin/pangolin.h>

int main(int /*argc*/, char** /*argv*/) {
    // 创建名称为“Main”的GUI窗口，尺寸为 800 × 600
    pangolin::CreateWindowAndBind("Main", 800, 600);
    // 启动深度测试
    glEnable(GL_DEPTH_TEST);

    // 创建一个观察相机视图,观察场景的相机
    // 第一行参数 宽高设置为窗口宽高，然后是内参fx fy，一般给个差不多的，然后是cx cy，给窗口的一半，最后是最近和最远距离
    // 第二行参数 相机所在的位置、相机看向的点的坐标，相机的轴的方向
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(800, 600, 500, 500, 400, 300, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -4, 2, 0, 0, 0, pangolin::AxisY));

    // 创建交互视图
    pangolin::Handler3D handler(s_cam); //交互相机视图句柄
    pangolin::View& d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, -800.0f / 600.0f)
                                .SetHandler(&handler);

    while (!pangolin::ShouldQuit()) {
        // 清空颜色和深度缓存
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glColor3f(0.5, 0.5, 0.5);
        glLineWidth(0.5);
        pangolin::glDraw_z0(0.5f, 10);

        // 在原点绘制一个立方体
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

        // 运行帧循环以推进窗口事件
        pangolin::FinishFrame();
    }

    return 0;
}
