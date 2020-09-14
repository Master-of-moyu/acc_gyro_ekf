#include <pangolin/pangolin.h>
#include <iostream>
#include <string>

void SampleMethod() {
    std::cout << "You typed ctrl-r or pushed reset" << std::endl;
}

int main(/*int argc, char* argv[]*/) {
    // 创建视窗
    pangolin::CreateWindowAndBind("Main", 800, 600);
    // 启动深度测试
    glEnable(GL_DEPTH_TEST);
    // 创建一个摄像机
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(800, 600, 500, 500, 400, 300, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -4, 2, 0, 0, 0, pangolin::AxisY));

    // 分割视窗
    const int UI_WIDTH = 220;
    // 右侧用于显示视口
    pangolin::View& d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -800.0f / 600.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));
    // 左侧用于创建控制面板
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));

    // 创建控制面板的控件对象，pangolin中
    pangolin::Var<bool> A_Button("ui.a_button", false, false);    // 按钮
    pangolin::Var<bool> A_Checkbox("ui.a_checkbox", false, true); // 选框
    pangolin::Var<double> Double_Slider("ui.a_slider", 3, 0, 5);  // double滑条
    pangolin::Var<int> Int_Slider("ui.b_slider", 2, 0, 5);        // int滑条
    pangolin::Var<std::string> A_string("ui.a_string", "Hello Pangolin");

    pangolin::Var<bool> SAVE_IMG("ui.save_img", false, false);     // 按钮
    pangolin::Var<bool> SAVE_WIN("ui.save_win", false, false);     // 按钮
    pangolin::Var<bool> RECORD_WIN("ui.record_win", false, false); // 按钮

    pangolin::Var<std::function<void()>> reset("ui.Reset", SampleMethod); //

    // 绑定键盘快捷键
    // Demonstration of how we can register a keyboard hook to alter a Var
    pangolin::RegisterKeyPressCallback(
        pangolin::PANGO_CTRL + 'b',
        pangolin::SetVarFunctor<double>("ui.a_slider", 3.5));

    // Demonstration of how we can register a keyboard hook to trigger a method
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'r', SampleMethod);
    // Default hooks for exiting (Esc) and fullscreen (tab).
    while (!pangolin::ShouldQuit()) {
        // Clear entire screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 各控件的回调函数
        if (pangolin::Pushed(A_Button))
            std::cout << "Push button A." << std::endl;

        if (A_Checkbox)
            Int_Slider = Double_Slider;
        // 保存整个win
        if (pangolin::Pushed(SAVE_WIN))
            pangolin::SaveWindowOnRender("window");
        // 保存view
        if (pangolin::Pushed(SAVE_IMG))
            d_cam.SaveOnRender("cube");
        // 录像
        if (pangolin::Pushed(RECORD_WIN))
            pangolin::DisplayBase().RecordOnRender(
                "ffmpeg:[fps=50,bps=8388608,unique_filename]//screencap.avi");
        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        glColor3f(0.5, 0.5, 0.5);
        glLineWidth(0.5);
        pangolin::glDraw_z0(0.5f, 10);

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

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return 0;
}
