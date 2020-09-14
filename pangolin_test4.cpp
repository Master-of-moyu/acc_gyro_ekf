#include <iostream>
#include <pangolin/pangolin.h>

int main(/*int argc, char* argv[]*/) {
    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind("Main", 640, 480);

    // Data logger object
    pangolin::DataLog log;

    // Optionally add named labels
    std::vector<std::string> labels;
    labels.push_back(std::string("sin(t)"));
    labels.push_back(std::string("cos(t)"));
    labels.push_back(std::string("sin(t)+cos(t)"));
    log.SetLabels(labels);

    const float tinc = 0.01f;

    // OpenGL 'view' of data. We might have many views of the same data.
    pangolin::Plotter plotter(&log, 0.0f, 4.0f * (float)M_PI / tinc, -4.0f, 4.0f, (float)M_PI / (4.0f * tinc), 0.5f);
    plotter.SetBounds(0.0, 1.0, 0.0, 1.0);
    plotter.Track("$i"); //坐标轴自动滚动

    // Add some sample annotations to the plot（为区域着色）
    // x < 50pi 的区域标记为蓝色
    plotter.AddMarker(pangolin::Marker::Vertical, 50 * M_PI, pangolin::Marker::LessThan, pangolin::Colour::Blue().WithAlpha(0.2f));
    // y > 3 的区域标记为红色
    plotter.AddMarker(pangolin::Marker::Horizontal, 3, pangolin::Marker::GreaterThan, pangolin::Colour::Red().WithAlpha(0.2f));
    // y = 3 的线标记为绿色
    plotter.AddMarker(pangolin::Marker::Horizontal, 3, pangolin::Marker::Equal, pangolin::Colour::Green());

    pangolin::DisplayBase().AddDisplay(plotter);

    float t = 0;

    // Default hooks for exiting (Esc) and fullscreen (tab).
    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        log.Log(sin(t), cos(t), sin(t) + cos(t));
        t += tinc;

        // Render graph, Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return 0;
}
