#include "vh_appsink.hpp"

#include <cstddef>
#include <iostream>
#include <thread>

#include "gst/gstbus.h"
#include "gst/gstelement.h"

const char *__THREAD_NAME = "TestAppLoop";
cv::Mat test_image;
using namespace cv;

TestApp::TestApp(int argc, char *argv[]) {
  gst_init(&argc, &argv);
  std::cout << "Gstreamer started!" << std::endl;

  // Create the elements
  pipeline = gst_parse_launch(GST_USER_PIPELINE, NULL);
  state = TestAppState::STOPPED;
  if (pipeline == NULL) {
    std::cout << "parse launch failed!" << std::endl;
    return;
  }

  appsink = gst_bin_get_by_name(GST_BIN(pipeline), "myappsink");
  // callback for new sample
  g_signal_connect(appsink, "new-sample", G_CALLBACK(TestApp::cb_new_sample),
                   this);

  // callback bus message
  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  gst_bus_add_watch(bus, (GstBusFunc)TestApp::cb_bus_message, this);

  loop = g_main_loop_new(NULL, FALSE);

  state = TestAppState::INIT;
}

TestApp::~TestApp() {
  std::cout << "Gstreamer try release!" << std::endl;
  gst_element_set_state(pipeline, GST_STATE_NULL);
  g_main_loop_quit(loop);
  gst_object_unref(pipeline);
  g_main_loop_unref(loop);
  std::cout << "gstreamer released!" << std::endl;
}

void TestApp::play() {
  std::cout << "TestAapp playing" << std::endl;
  state = TestAppState::PLAYING;
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
}

void TestApp::pause() {
  std::cout << "TestApp paused" << std::endl;
  state = TestAppState::PAUSED;
  gst_element_set_state(pipeline, GST_STATE_PAUSED);
}

void TestApp::exec() {
  std::cout << "TestApp run loop" << std::endl;
  const char *name = __THREAD_NAME;
  std::thread gst_main_loop([name, this]() { g_main_loop_run(this->loop); });
  gst_main_loop.detach();
  this->play();
}

GstFlowReturn TestApp::cb_new_sample(GstElement *sink, TestApp *app) {
  (void)app;
  GST_TRACE("start cb_new_sample");
  GstSample *sample;
  //   GstBuffer *app_buffer;
  GstBuffer *buffer;
  //   GstElement *source;
  //   GstFlowReturn ret;

  /* get the sample from appsink */
  g_signal_emit_by_name(sink, "pull-sample", &sample);
  if (!sample) {
    return GST_FLOW_ERROR;
  }
  buffer = gst_sample_get_buffer(sample);
  if (!buffer) {
    return GST_FLOW_ERROR;
  }

  /* process sample */
  GstMapInfo info = GST_MAP_INFO_INIT;
  if (!gst_buffer_map(buffer, &info, GST_MAP_READ)) {
    return GST_FLOW_ERROR;
  }

  cb_mjpeg((const char *)info.data, info.size);

  gst_buffer_unmap(buffer, &info);
  /* we don't need the appsink sample anymore */
  gst_sample_unref(sample);
  GST_TRACE("end cb_new_sample");
  return GST_FLOW_OK;
}

gboolean TestApp::cb_bus_message(GstBus *bus, GstMessage *message,
                                 TestApp *app) {
  (void)bus;
  (void)app;
  GST_DEBUG("got message %s",
            gst_message_type_get_name(GST_MESSAGE_TYPE(message)));
  GError *info = NULL;
  gchar *debug = NULL;
  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR:
      gst_message_parse_error(message, &info, &debug);
      GST_ERROR("received error: info: %s, debug: %s", info->message, debug);
      if (info != NULL) {
        g_error_free(info);
        info = NULL;
      }
      if (debug != NULL) {
        g_free(debug);
        debug = NULL;
      }
      break;
    case GST_MESSAGE_EOS:
      GST_ERROR("received EOS");
      break;
    default:
      break;
  }
  return TRUE;
}

void TestApp::cb_mjpeg(const char *const data, size_t size) {
  cv::Mat rawData(1, size, CV_8UC1, (void *)data);
  test_image = cv::imdecode(rawData, 1);
  // if (this->image.data == NULL)
  // {
  //   // Error reading raw image data
  // }
  // imshow("wat", test_image);
  // std::cout << test_image.size << std::endl;
//   std::cout << int(test_image.data[0]) << std::endl;
}

cv::Mat TestApp::get_image() {
//   std::unique_lock<std::mutex> lck(mtx, std::defer_lock);
//   lck.lock();
  if (test_image.data != NULL) {
    this->image = test_image;
    std::cout << int(image.data[0]) << std::endl;
  }

//   lck.unlock();
  return this->image;
}
