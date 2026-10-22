// Standalone Spinnaker stereo viewer; launched by stereo_camera_gui.sh.
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <array>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstdlib>

namespace fs = std::filesystem;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
static const char* view = "Stereo: left | right";
static const char* controls = "Stereo controls";

bool windowClosed(const char* name, bool& was_visible) {
    // Some HighGUI backends report -1 (unsupported), and forwarded X11 windows
    // may report 0 while being mapped. Neither means a new window was closed.
    try {
        const double visible = cv::getWindowProperty(name,cv::WND_PROP_VISIBLE);
        if (visible >= 1) was_visible = true;
        return was_visible && visible < 1;
    } catch (const cv::Exception&) {
        // Qt can throw when a previously visible window has been destroyed.
        return was_visible;
    }
}

int environmentInteger(const char* name, int fallback, int minimum, int maximum) {
    const char* raw = std::getenv(name);
    if (!raw) return fallback;
    try {
        size_t used = 0;
        int result = std::stoi(raw, &used);
        if (used == std::string(raw).size() && result >= minimum && result <= maximum)
            return result;
    } catch (const std::exception&) {}
    throw std::runtime_error(std::string(name)+" must be an integer between "+
                             std::to_string(minimum)+" and "+std::to_string(maximum));
}

void enumeration(CameraPtr c, const char* name, const char* value) {
    CEnumerationPtr n = c->GetNodeMap().GetNode(name);
    if (!IsWritable(n)) throw std::runtime_error(std::string(name)+" is unavailable or locked");
    auto e = n->GetEntryByName(value);
    if (!IsReadable(e)) throw std::runtime_error(std::string(name)+" does not support "+value);
    n->SetIntValue(e->GetValue());
}
void number(CameraPtr c, const char* name, double value) {
    CFloatPtr n = c->GetNodeMap().GetNode(name);
    if (!IsWritable(n)) throw std::runtime_error(std::string(name)+" is unavailable or locked");
    n->SetValue(std::clamp(value, n->GetMin(), n->GetMax()));
}
void integer(CameraPtr c, const char* name, bool maximum) {
    CIntegerPtr n = c->GetNodeMap().GetNode(name);
    if (IsWritable(n)) n->SetValue(maximum ? n->GetMax() : n->GetMin());
}
template<class T> T value(YAML::Node n, const char* key, T fallback) {
    return n[key] ? n[key].as<T>() : fallback;
}
struct Session {
    SystemPtr system = System::GetInstance();
    CameraList list = system->GetCameras();
    std::array<CameraPtr, 2> cameras;
    ~Session() {
        for (auto& c : cameras) {
            if (!c) continue;
            try { if (c->IsStreaming()) c->EndAcquisition(); } catch (...) {}
            try { if (c->IsInitialized()) c->DeInit(); } catch (...) {}
            c = nullptr;
        }
        list.Clear(); system->ReleaseInstance();
    }
};
static bool save_requested = false;
void mouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN && x >= 10 && x <= 290 && y >= 15 && y <= 65)
        save_requested = true;
}
std::string filename(char prefix, unsigned index) {
    std::ostringstream s; s << prefix << std::setw(3) << std::setfill('0') << index << ".png";
    return s.str();
}
void savePair(const fs::path& root, const std::array<cv::Mat,2>& pair, unsigned& index) {
    fs::create_directories(root/"left"); fs::create_directories(root/"right");
    while (fs::exists(root/"left"/filename('L',index)) || fs::exists(root/"right"/filename('R',index))) ++index;
    auto l = root/"left"/filename('L',index), r = root/"right"/filename('R',index);
    // Exclusive creation prevents replacing an existing capture, even with another viewer running.
    auto write = [](const fs::path& p, const cv::Mat& image) {
        std::vector<uchar> bytes;
        if (!cv::imencode(".png", image, bytes)) throw std::runtime_error("PNG encoding failed");
        FILE* f = fopen(p.c_str(), "wbx");
        if (!f) throw std::runtime_error("Cannot create "+p.string());
        bool ok = fwrite(bytes.data(),1,bytes.size(),f) == bytes.size();
        if (fclose(f) != 0) ok = false;
        if (!ok) { fs::remove(p); throw std::runtime_error("Cannot write "+p.string()); }
    };
    write(l,pair[0]);
    try { write(r,pair[1]); } catch (...) { fs::remove(l); throw; }
    std::cout << "Saved " << l << " and " << r << '\n'; ++index;
}
int main(int argc, char** argv) {
    if (argc < 2 || std::string(argv[1]) == "--help") {
        std::cout << "Usage: stereo_camera_gui.sh CONFIG.yaml [OUTPUT_FOLDER]\n"
                     "S / Save button: save pair; F: toggle fullscreen; Q / Esc: quit.\n";
        return argc < 2 ? 1 : 0;
    }
    try {
        const int preview_width = environmentInteger("STEREO_GUI_PREVIEW_WIDTH", 0, 0, 16384);
        const int preview_fps = environmentInteger("STEREO_GUI_PREVIEW_FPS", 30, 1, 120);
        bool fullscreen = environmentInteger("STEREO_GUI_FULLSCREEN", 1, 0, 1) != 0;
        auto config = YAML::LoadFile(argv[1]);
        if (config["passive_stereo_node"]) config = config["passive_stereo_node"]["ros__parameters"];
        fs::path output = argc > 2 ? argv[2] : value<std::string>(config,"capture_folder","stereo_pairs");
        std::array<std::string,2> serials {config["cam_left_serial"].as<std::string>(), config["cam_right_serial"].as<std::string>()};
        if (serials[0] == serials[1]) throw std::runtime_error("Left and right serials must differ");
        Session session;
        for (int side=0; side<2; ++side) {
            session.cameras[side] = session.list.GetBySerial(serials[side]);
            if (!session.cameras[side]) throw std::runtime_error("Camera not found: "+serials[side]);
            auto c = session.cameras[side]; c->Init();
            enumeration(c,"TriggerMode","Off");
            enumeration(c,"AcquisitionMode","Continuous");
            enumeration(c,"ExposureMode","Timed");
            enumeration(c,"ExposureAuto","Off"); enumeration(c,"GainAuto","Off");
            integer(c,"BinningHorizontal",false); integer(c,"BinningVertical",false);
            integer(c,"DecimationHorizontal",false); integer(c,"DecimationVertical",false);
            integer(c,"OffsetX",false); integer(c,"OffsetY",false);
            integer(c,"Width",true); integer(c,"Height",true);
            CBooleanPtr rate = c->GetNodeMap().GetNode("AcquisitionFrameRateEnable");
            if (IsWritable(rate)) { rate->SetValue(true); number(c,"AcquisitionFrameRate",value<double>(config,"frame_rate",30)); }
            CEnumerationPtr buffer = c->GetTLStreamNodeMap().GetNode("StreamBufferHandlingMode");
            if (IsWritable(buffer)) { auto newest=buffer->GetEntryByName("NewestOnly"); if(IsReadable(newest)) buffer->SetIntValue(newest->GetValue()); }
        }
        int exposure=std::lround(value<double>(config,"exposure_time",15000));
        int gain=std::lround(value<double>(config,"gain",0)*10);
        int red=std::lround(value<double>(config,"balance_ratio_red",1.5)*100);
        int blue=std::lround(value<double>(config,"balance_ratio_blue",1.5)*100);
        const char* modes[] = {"Off","Once","Continuous"};
        auto mode = [&](const char* key, int fallback) {
            std::string s=value<std::string>(config,key,modes[fallback]);
            for(int i=0;i<3;++i) if(s==modes[i]) return i;
            throw std::runtime_error(std::string(key)+" must be Off, Once, or Continuous");
        };
        int exp_mode=mode("exposure_auto",0), gain_mode=mode("gain_mode",value<bool>(config,"gain_auto",false)?2:0);
        int wb_mode=mode("white_balance_mode",value<bool>(config,"balance_white_auto",true)?2:0);
        cv::namedWindow(view,cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        if (fullscreen) cv::setWindowProperty(view,cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);
        else cv::resizeWindow(view,preview_width > 0 ? preview_width : 1280,480);
        cv::namedWindow(controls,cv::WINDOW_NORMAL); cv::resizeWindow(controls,650,550);
        auto track = [&](const char* name, int& initial, int max) {
            initial=std::clamp(initial,0,max);
            cv::createTrackbar(name,controls,nullptr,max); cv::setTrackbarPos(name,controls,initial);
        };
        // Ranges shared by both cameras; actual float bounds are also checked on every write.
        double exp_max=1000000, gain_max=100, wb_max=10;
        for(auto c:session.cameras) {
            CFloatPtr e=c->GetNodeMap().GetNode("ExposureTime"), g=c->GetNodeMap().GetNode("Gain");
            exp_max=std::min(exp_max,e->GetMax()); gain_max=std::min(gain_max,g->GetMax());
            CFloatPtr w=c->GetNodeMap().GetNode("BalanceRatio"); if(IsReadable(w)) wb_max=std::min(wb_max,w->GetMax());
        }
        track("Exposure us",exposure,int(exp_max)); track("Gain x0.1 dB",gain,int(gain_max*10));
        track("Exposure auto",exp_mode,2); track("Gain auto",gain_mode,2);
        track("White balance auto",wb_mode,2); track("Red x0.01",red,int(wb_max*100)); track("Blue x0.01",blue,int(wb_max*100));
        cv::setMouseCallback(controls,mouse);
        std::array<int,7> previous {-1,-1,-1,-1,-1,-1,-1};
        std::string status="Ready. Modes: 0 manual, 1 once, 2 continuous";
        std::array<cv::Mat,2> pair;
        ImageProcessor processor; unsigned index=0;
        using Clock = std::chrono::steady_clock;
        auto next_preview = Clock::now();
        const auto preview_interval = std::chrono::microseconds(1000000 / preview_fps);
        std::string displayed_status;
        std::cout << "Capture folder (on this computer): " << fs::absolute(output) << '\n'
                  << "Preview: " << preview_fps << " FPS, width " << preview_width
                  << " (0 = native). Saved images retain native resolution.\n";
        auto apply = [&] {
            std::array<int,7> now {cv::getTrackbarPos("Exposure us",controls),cv::getTrackbarPos("Gain x0.1 dB",controls),cv::getTrackbarPos("Exposure auto",controls),cv::getTrackbarPos("Gain auto",controls),cv::getTrackbarPos("White balance auto",controls),cv::getTrackbarPos("Red x0.01",controls),cv::getTrackbarPos("Blue x0.01",controls)};
            if(now==previous) return;
            for (int side=0;side<2;++side) {
                auto c=session.cameras[side];
                try {
                    if(now[2]!=previous[2]) enumeration(c,"ExposureAuto",modes[now[2]]);
                    if(now[3]!=previous[3]) enumeration(c,"GainAuto",modes[now[3]]);
                    if(now[2]==0 && (now[0]!=previous[0] || now[2]!=previous[2])) number(c,"ExposureTime",now[0]);
                    if(now[3]==0 && (now[1]!=previous[1] || now[3]!=previous[3])) number(c,"Gain",now[1]/10.0);
                    if(now[4]!=previous[4]) enumeration(c,"BalanceWhiteAuto",modes[now[4]]);
                    if(now[4]==0) {
                        if(now[5]!=previous[5] || now[4]!=previous[4]) { enumeration(c,"BalanceRatioSelector","Red"); number(c,"BalanceRatio",now[5]/100.0); }
                        if(now[6]!=previous[6] || now[4]!=previous[4]) { enumeration(c,"BalanceRatioSelector","Blue"); number(c,"BalanceRatio",now[6]/100.0); }
                    }
                    status="Applied controls to both cameras";
                } catch(const std::exception& e) { status=serials[side]+": "+e.what(); std::cerr<<status<<'\n'; }
            }
            previous=now;
        };
        apply();
        for(auto c:session.cameras) c->BeginAcquisition();
        int failures=0;
        bool view_was_visible=false, controls_were_visible=false;
        while(true) {
            apply(); bool complete=true;
            for(int side=0;side<2;++side) {
                ImagePtr raw;
                try {
                    raw=session.cameras[side]->GetNextImage(200);
                    if(raw->IsIncomplete()) complete=false;
                    else { auto bgr=processor.Convert(raw,PixelFormat_BGR8); pair[side]=cv::Mat(bgr->GetHeight(),bgr->GetWidth(),CV_8UC3,bgr->GetData(),bgr->GetStride()).clone(); }
                    raw->Release(); raw=nullptr;
                } catch(const std::exception& e) { if(raw) raw->Release(); complete=false; status=e.what(); }
            }
            if(complete) {
                failures=0;
                if(pair[0].size()!=pair[1].size()) throw std::runtime_error("Camera image dimensions differ");
                if (Clock::now() >= next_preview) {
                    cv::Mat stereo;
                    if (preview_width > 0 && pair[0].cols * 2 > preview_width) {
                        // Resize before sending pixels through X11; keep native images for saving.
                        std::array<cv::Mat,2> small;
                        const int width = std::max(1, preview_width / 2);
                        const int height = std::max(1, int(std::lround(double(pair[0].rows) * width / pair[0].cols)));
                        for (int side=0;side<2;++side)
                            cv::resize(pair[side],small[side],{width,height},0,0,cv::INTER_AREA);
                        cv::hconcat(small[0],small[1],stereo);
                    } else cv::hconcat(pair[0],pair[1],stereo);
                    cv::imshow(view,stereo);
                    next_preview = Clock::now() + preview_interval;
                }
                if(save_requested) { try { savePair(output,pair,index); status="Saved pair "+std::to_string(index-1)+" to "+output.string(); } catch(const std::exception& e) {status=e.what();} save_requested=false; }
            } else if(++failures>=30) throw std::runtime_error("Repeated capture failures: "+status);
            if (status != displayed_status) {
                cv::Mat panel(150,650,CV_8UC3,cv::Scalar(35,35,35));
                cv::rectangle(panel,{10,15},{290,65},cv::Scalar(60,140,60),cv::FILLED);
                cv::putText(panel,"SAVE IMAGE PAIR (S)",{20,47},cv::FONT_HERSHEY_SIMPLEX,.65,{255,255,255},1);
                cv::putText(panel,"0: manual   1: once   2: continuous",{10,95},cv::FONT_HERSHEY_SIMPLEX,.55,{255,255,255},1);
                cv::putText(panel,status.substr(0,85),{10,125},cv::FONT_HERSHEY_SIMPLEX,.4,{255,255,255},1);
                cv::imshow(controls,panel);
                displayed_status = status;
            }
            int key=cv::waitKey(1)&255;
            if(key==27 || key=='q') {
                std::cout << "Closing viewer: Q/Esc pressed.\n";
                break;
            }
            if(windowClosed(view,view_was_visible) || windowClosed(controls,controls_were_visible)) {
                std::cout << "Closing viewer: a previously visible window was closed.\n";
                break;
            }
            if(key=='s') save_requested=true;
            if(key=='f') { fullscreen=!fullscreen; cv::setWindowProperty(view,cv::WND_PROP_FULLSCREEN,fullscreen?cv::WINDOW_FULLSCREEN:cv::WINDOW_NORMAL); }
        }
        cv::destroyAllWindows();
        return 0;
    } catch(const std::exception& e) { std::cerr<<"Stereo viewer: "<<e.what()<<'\n'; return 1; }
}
