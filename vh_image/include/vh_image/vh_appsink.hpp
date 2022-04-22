#ifndef __VH_APPSINK_HPP__
#define __VH_APPSINK_HPP__

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "gst/gstbus.h"
#include "gst/gstmessage.h"
// #include <opencv2/cudaimgproc.hpp>
#include <mutex>

extern "C" {
#include <gst/gst.h>
}

#define GST_USER_PIPELINE                                                      \
  "shmsrc socket-path=/tmp/livesrc_XIASHI_L is-live=true do-timestamp=true ! " \
  "jpegparse ! appsink name=myappsink emit-signals=true sync=false"

// class TestApp;
extern cv::Mat test_image;

class TestApp {
 public:
  enum class TestAppState { INIT, PLAYING, PAUSED, STOPPED };

 private:
  GstElement *pipeline;
  GstElement *appsink;
  GMainLoop *loop;
  GstBus *bus;
  TestAppState state;

 public:
  TestApp(int argc, char *argv[]);
  ~TestApp();
  void exec();
  void play();
  void pause();
  cv::Mat get_image();

 private:
  static GstFlowReturn cb_new_sample(GstElement *appsink, TestApp *app);
  static gboolean cb_bus_message(GstBus *bus, GstMessage *message,
                                 TestApp *app);
  // comsume data here
  static void cb_mjpeg(const char *const data, const size_t size);
  cv::Mat image;
  std::mutex mtx;
};

#endif