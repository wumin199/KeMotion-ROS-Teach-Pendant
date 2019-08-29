
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "voice_control/qisr.h"
#include "voice_control/msp_cmn.h"
#include "voice_control/msp_errors.h"
#include "voice_control/speech_recognizer.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_srvs/SetBool.h>
#include <string>

#define FRAME_LEN	640
#define	BUFFER_SIZE	4096

ros::Publisher status_pub_;

bool startFlag = false;//set by ui
int resultFlag = 0;

static void show_result(char *str_speech, char is_over)
{
  printf("this is show_result\n");
  //if no speech or error occurs, show_result will not be called
  std::string str(str_speech);
  int flag = is_over;
  printf("show result is_over = [%d]\n", is_over);
  std::cout<< "\rResult\t["<< str<< "] " <<"is_over = [" <<flag<<"]" <<std::endl;

  /*
  if (str.compare("hello robot"))
  {
    resultFlag = 0;
    return;
  }

  if (str.compare("move home"))
  {
    resultFlag = 1;
  }

  if (str.compare("move up"))
  {
    resultFlag = 1;
  }

  if (str.compare("move down"))
  {
    resultFlag = 1;
  }

  if (str.compare("move left"))
  {
    resultFlag = 1;
  }

  if (str.compare("move right"))
  {
    resultFlag = 1;
  }
  */


  // printf("\rResult: [ %s ]", str_speech);
  //if(is_over)
  //  putchar('\n');
  if (is_over)
  //if (flag == 1)
  {
    std::cout<<"pub status"<<std::endl;
    std_msgs::String msg;
    if (str !="")
    {
      msg.data = str;
    }
    else {
      msg.data = "didn't hear anything";
    }
    status_pub_.publish(msg);
  }
  else {
    std::cout<<"flag is " << flag << std::endl;
  }

}

static char *g_result = NULL;//audio result
static unsigned int g_buffersize = BUFFER_SIZE;

/**
 * @brief called when speech recognized
 * @param result
 * @param is_last
 */
void on_result(const char *result, char is_last)
{
  if (result) {
    size_t left = g_buffersize - 1 - strlen(g_result);
    size_t size = strlen(result);
    if (left < size) {
      g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
      if (g_result)
        g_buffersize += BUFFER_SIZE;
      else {
        printf("mem alloc failed\n");
        return;
      }
    }
    printf("on_result\n");
    printf("is_last = [%d]\n", is_last);
    strncat(g_result, result, size);
    show_result(g_result, is_last);
  }
}
void on_speech_begin()
{
  if (g_result)
  {
    free(g_result);
  }
  g_result = (char*)malloc(BUFFER_SIZE);
  g_buffersize = BUFFER_SIZE;
  memset(g_result, 0, g_buffersize);

  printf("Start Listening...\n");
}

/**
 * @brief called when vad_eos=2000, vad_bos=10000 reached
 * @param reason
 */
void on_speech_end(int reason)
{
  printf("on_speed_end\n");

  if (reason == END_REASON_VAD_DETECT)
    printf("\nSpeaking done \n");
  else
    printf("\nRecognizer error %d\n", reason);
}

/* demo recognize the audio from microphone in 10s */
static void demo_mic(const char* session_begin_params)
{
  int errcode;
  int i = 0;

  struct speech_rec iat;

  struct speech_rec_notifier recnotifier = {
    on_result,
    on_speech_begin,
    on_speech_end
  };

  errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
  if (errcode) {
    printf("speech recognizer init failed\n");
    return;
  }
  errcode = sr_start_listening(&iat);
  if (errcode) {
    printf("start listen failed %d\n", errcode);
  }
  /* demo 10 seconds recording */

  while (startFlag)
  {
    sleep(0.4);
  }
  printf("out while \n");
  /*
  while(i++ < 20)
    sleep(1);
  */
  errcode = sr_stop_listening(&iat);
  if (errcode) {
    printf("stop listening failed %d\n", errcode);
  }

  sr_uninit(&iat);
}

bool voice_control(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data == true)
  {
    startFlag = true;
    printf("startFlag = true\n");
  }
  else
  {
    startFlag = false;
    printf("startFlag = false\n");
  }
  res.success = true;
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ArmVoiceControl");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  std::string lang = "zh_cn";
  ros::param::get("/arm_voice_control/language", lang);

  status_pub_ = n.advertise<std_msgs::String>("rtp_node/speech_status", 1000);
  ros::ServiceServer voice_control_server_ = n.advertiseService("rtp_node/voice_control", voice_control);

  ros::AsyncSpinner common_spinner(4);//async process
  common_spinner.start();

  while (ros::ok())
  {
    if (startFlag)
    {
      int ret = MSP_SUCCESS;

      const char* login_params = "appid = 5d48d7a3, work_dir = .";

      const char* session_begin_params = (lang == "zh_cn")?
              "sub = iat, domain = iat, language = zh_cn, "
              "accent = mandarin, sample_rate = 16000, "
              "result_type = plain, result_encoding = utf8, "
              "vad_enable=true, vad_eos=2000, vad_bos=10000":
            "sub = iat, domain = iat, language = en_us, "
            "accent = mandarin, sample_rate = 16000, "
            "result_type = plain, result_encoding = utf8, "
            "vad_enable=true, vad_eos=2000, vad_bos=10000";

      ret = MSPLogin(NULL, NULL, login_params);
      if (MSP_SUCCESS != ret)
      {
        MSPLogout();
        printf("MSPLogin failed , Error code %d.\n",ret);
      }
      demo_mic(session_begin_params);
      MSPLogout();
      printf("end recording...\n");
      if (resultFlag)
      {
        resultFlag = 0;
        printf("result flag\n");
      }

    }

    ros::spinOnce();
    loop_rate.sleep();
  }


}
