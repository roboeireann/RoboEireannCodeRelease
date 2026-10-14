/**
 * @file Main.cpp
 * Implementation of the main() function for starting and stopping the module framework.
 * @author Colin Graf
 */

#include "Framework/Robot.h"
#include "Framework/Robots.h"
#include "Framework/Settings.h"
#include "Math/Angle.h"
#include "Math/Constants.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "RobotParts/Joints.h"
#include "Streaming/FunctionList.h"
#include "Streaming/InStreams.h"
#include "Streaming/OutStreams.h"
#include "Tools/Communication/MsgPack.h"
#include "TextLogging/TextLogging.h"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <chrono>

#include <sys/file.h> // flock
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>


static pid_t roboeireannPid = 0;
static Robot* robot = nullptr;
static bool run = true;
static pthread_t mainThread;
static bool shutdownNAO = false;


DECL_TLOGGER(tlogger, "Main", TextLogging::INFO);


SystemCall::Mode SystemCall::getMode()
{
  return physicalRobot;
}

static void roboeireannStart(const Settings& settings)
{
  TLOGI(tlogger, "RoboEireann: Start.");

  robot = new Robot(settings, std::string());
  robot->start();
}

static void roboeireannStop()
{
  TLOGI(tlogger, "RoboEireann: Stop.");

  robot->announceStop();
  robot->stop();
  delete robot;
  robot = nullptr;
}

static void sighandlerShutdown(int sig)
{
  if(pthread_self() != mainThread)
  {
    // When we detect a long chest button press, the thread on which it is detected raises
    // SIGINT and will be the thread to initially catch the signal, hence we need to redirect
    // to the main thread here.
    TLOGI(tlogger, "Caught termination/interrupt signal ({}:{}) on non-main thread -- redirecting to main thread...",
          sig, Assert::getSignalName(sig));

    shutdownNAO = true;
    pthread_kill(mainThread, sig);
  }
  else
  {
    if(run)
    {
      TLOGI(tlogger, "Caught normal termination/interrupt signal ({}:{}) -- Shutting down executable...",
            sig, Assert::getSignalName(sig));
      TextLogging::requestAsyncOutput(false); // output any async log messages
      fflush(NULL); // flush all streams
    }
    
    run = false;
  }
}

static void sighandlerRedirect(int sig)
{
  Thread::sleep(100);
  TLOGI(tlogger, "sighandlerRedirect, caught {}, set run to false", Assert::getSignalName(sig));
  TextLogging::requestAsyncOutput(false); // output any async log messages
  fflush(NULL); // try to flush all streams

  run = false;
}

static std::string getBodyId()
{
  int socket = ::socket(AF_UNIX, SOCK_STREAM, 0);
  sockaddr_un address;
  address.sun_family = AF_UNIX;
  std::strcpy(address.sun_path, "/tmp/robocup");
  if(connect(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)))
  {
    TLOGI(tlogger, "Waiting for LoLA...");
    while(connect(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)))
      usleep(100000);
    TLOGI(tlogger, "...LoLA OK");
  }

  std::string bodyId;

  // Receive a single packet and extract serial number of body.
  unsigned char receivedPacket[896];
  long bytesReceived = recv(socket, reinterpret_cast<char*>(receivedPacket), sizeof(receivedPacket), 0);
  if(bytesReceived >= 0)
    MsgPack::parse(receivedPacket, bytesReceived,
                   [](const std::string&, const unsigned char*) {},
                   [](const std::string&, const unsigned char*) {},
                   [&bodyId](const std::string& key, const unsigned char* value, size_t valueSize)
                   {
                     if(key == "RobotConfig:0" && valueSize)
                     {
                       bodyId.resize(valueSize);
                       std::strncpy(&bodyId[0], reinterpret_cast<const char*>(value), valueSize);
                     }
                   });
  close(socket);

  return bodyId;
}

static void sitDown(bool ok)
{
  TLOGI(tlogger, "sit down the Nao if needed, normalShutdown {}", ok);

  static constexpr Joints::Joint numOfJoints = static_cast<Joints::Joint>(Joints::numOfJoints - 2);
  static const Angle targetAngles[numOfJoints] =
  {
    0_deg, 0_deg, // Head

    51_deg, 3_deg, 15_deg, -36_deg, -90_deg,  // Left arm
    0_deg, 0_deg, -50_deg, 124_deg, -68_deg, 0_deg, // HipYawPitch and left leg

    0_deg, -50_deg, 124_deg, -68_deg, 0_deg, // Right leg
    51_deg, -3_deg, -15_deg, 36_deg, 90_deg, // Right arm

    0_deg, 0_deg // Hands
  };

  int socket = ::socket(AF_UNIX, SOCK_STREAM, 0);
  sockaddr_un address;
  address.sun_family = AF_UNIX;
  std::strcpy(address.sun_path, "/tmp/robocup");
  if(!connect(socket, reinterpret_cast<sockaddr*>(&address), sizeof(address)))
  {
    unsigned char packet[896];
    long bytesReceived = recv(socket, reinterpret_cast<char*>(packet), static_cast<int>(sizeof(packet)), 0);

    // Determine current angles and whether sitting down is required, i.e. has the hip stiffness?
    bool sitDownRequired = false;
    Angle startAngles[numOfJoints];
    std::array<float, 3> torsoAngles; /**< The addresses of torso angle data inside receivedPacket. */

    MsgPack::parse(packet, bytesReceived,
                   [&sitDownRequired, &startAngles](const std::string& key, const unsigned char* p)
                   {
                     const std::string::size_type pos = key.find(":");
                     ASSERT(pos != std::string::npos);
                     const std::string category = key.substr(0, pos);
                     const int index = std::stoi(key.substr(pos + 1));

                     if(category == "Position")
                       startAngles[index] = MsgPack::readFloat(p);
                     else if(category == "Stiffness" && (index == 8 || index == 13) && MsgPack::readFloat(p) > 0.f)
                       sitDownRequired = true;
                   },

                   // Ignore ints and strings
                   [](const std::string&, const unsigned char*) {},
                   [](const std::string&, const unsigned char*, size_t) {});

    // If sitting down is required, interpolate from start angles to target angles.
    if(sitDownRequired)
    {
      TLOGI(tlogger, "...sit down is required");

      float ratio = 0.f;
      while(ratio < 1.f)
      {
        // Do sinus interpolation
        const float phase = 0.5f * std::sin(ratio * Constants::pi - Constants::pi / 2.f) + 0.5f;

        // Get torso angles from IMU
        MsgPack::parse(packet, bytesReceived,
                       [&torsoAngles](const std::string& key, const unsigned char* p)
                       {
                         const std::string::size_type pos = key.find(":");
                         ASSERT(pos != std::string::npos);
                         const std::string category = key.substr(0, pos);
                         const int index = std::stoi(key.substr(pos + 1));
                         if(category == "Angles")
                           torsoAngles[index] = MsgPack::readFloat(p);
                       },
                       // Ignore ints and strings
                       [](const std::string&, const unsigned char*) {},
                       [](const std::string&, const unsigned char*, size_t) {});

        unsigned char* p = packet;
        // Reduce stiffness if robot is falling
        if(std::abs(torsoAngles[0]) > 45_deg || std::abs(torsoAngles[1]) > 45_deg)
        {
          MsgPack::writeMapHeader(2, p);
          MsgPack::write("Stiffness", p);
          MsgPack::writeArrayHeader(numOfJoints, p);

          for(int i = 0; i < numOfJoints; ++i)
            MsgPack::write(0.2f, p);
        }
        else
          MsgPack::writeMapHeader(1, p);

        MsgPack::write("Position", p);
        MsgPack::writeArrayHeader(numOfJoints, p);
        // The shoulder pitch joints interpolate faster to avoid collisions of the arms with the legs.
        const float shoulderPitchPhase = std::sqrt(std::min(1.f, ratio / 0.6f));
        for(int i = 0; i < numOfJoints; ++i)
        {
          if(i == 2 || i == 18)
            MsgPack::write(targetAngles[i] * shoulderPitchPhase + startAngles[i] * (1.f - shoulderPitchPhase), p);
          else
            MsgPack::write(targetAngles[i] * phase + startAngles[i] * (1.f - phase), p);
        }

        // Send packet to LoLA
        send(socket, reinterpret_cast<char*>(packet), static_cast<int>(p - packet), 0);

        // Receive next packet (required for sending again)
        bytesReceived = recv(socket, reinterpret_cast<char*>(packet), static_cast<int>(sizeof(packet)), 0);

        ratio += Constants::naoMotionCycleTime / 2.f; // 2 seconds
      }
    }

    // Switch off stiffness of all joints
    unsigned char* p = packet;
    MsgPack::writeMapHeader(9, p);
    MsgPack::write("Stiffness", p);
    MsgPack::writeArrayHeader(numOfJoints, p);
    for(int i = 0; i < numOfJoints; ++i)
      MsgPack::write(0.f, p);

    // Switch off all leds, except for the eyes (ok: blue, crashed: red)
    static std::string ledCategories[8] = {"LEye", "REye", "LEar", "REar", "Skull", "Chest", "LFoot", "RFoot"};
    static size_t ledNumbers[8] = {24, 24, 10, 10, 12, 3, 3, 3};
    for(int i = 0; i < 8; ++i)
    {
      MsgPack::write(ledCategories[i], p);
      MsgPack::writeArrayHeader(ledNumbers[i], p);
      for(size_t j = 0; j < ledNumbers[i]; ++j)
      {
        bool isRed = i < 2 && j < 8;
        bool isBlue = i < 2 && j >= 16;
        MsgPack::write(ok && isBlue ? 0.1f : !ok && isRed ? 1.f : 0.f, p);
      }
    }

    // Send packet to LoLA
    send(socket, reinterpret_cast<char*>(packet), static_cast<int>(p - packet), 0);
  }
  close(socket);
}

static std::string getBodyName(const std::string& bodyId)
{
  std::string bodyName;
  std::string bhdir = File::getBHDir();
  InMapFile robotsStream(bhdir + "/Config/Robots/robots.cfg");
  if(!robotsStream.exists())
    TLOGW(tlogger, "Could not load robots.cfg");
  else
  {
    Robots robots;
    robotsStream >> robots;
    for(const Robots::RobotId& robot : robots.robotsIds)
      if(robot.robotType == Settings::nao && robot.bodyId == bodyId)
      {
        bodyName = robot.name;
        return bodyName;
      }

    TLOGW(tlogger, "Could not find bodyName in robots.cfg! BodyId: {} -- Assuming \"Default\".", bodyId);
    bodyName = "Default";
  }
  return bodyName;
}

int main(int argc, char* argv[])
{
  {
    // Set stdout to be unbuffered.
    // This has previously been done using stdbuf, but this does not work for a 64-bit program on a 32-bit system.
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    mainThread = pthread_self();
    Thread::nameCurrentThread("ParentMain"); // for text logging output

    // parse command-line arguments
    bool watchdog = false;
    const char* configDir = "/home/nao";

    for(int i = 1; i < argc; ++i)
      if(!strcmp(argv[i], "-c") && i + 1 < argc)
        configDir = argv[++i];
      else if(!strcmp(argv[i], "-w"))
        watchdog = true;
      else
      {
        TLOGI(tlogger, "\n  Usage: {} [-c <dir>] [-w]\n\
      -c <dir>      base directory for config (default is /home/nao)\n\
      -w            use a watchdog for crash recovery and creating trace dumps", argv[0]);

        exit(EXIT_FAILURE);
      }

    TLOGI(tlogger, "Starting up, watchdog={}", watchdog);

    // avoid duplicated instances
    int fd = open("/tmp/roboeireann", O_CREAT, 0600);
    if(fd == -1 || flock(fd, LOCK_EX | LOCK_NB) == -1)
    {
      TLOGE(tlogger, "There is already an instance of this process!");
      exit(EXIT_FAILURE);
    }

    // change working directory
    if(*configDir && chdir(configDir) != 0)
    {
      TLOGE(tlogger, "chdir to config directory ({}) failed!", configDir);
      exit(EXIT_FAILURE);
    }

    // the watchdog
    if(watchdog)
    {
      for(;;)
      {
        roboeireannPid = fork();
        if(roboeireannPid == -1)
          exit(EXIT_FAILURE);
        else if(roboeireannPid == 0)
          break; // child process

        // if we get this far we are the parent (watchdog) version of the process
        int status;
        signal(SIGTERM, sighandlerRedirect);
        signal(SIGINT, sighandlerRedirect);
        if(waitpid(roboeireannPid, &status, 0) != roboeireannPid)
        {
          exit(EXIT_FAILURE);
        }
        signal(SIGTERM, SIG_DFL);
        signal(SIGINT, SIG_DFL);

        // detect requested or normal exit
        bool normalExit = !run || (WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);

        TLOGI(tlogger, "Child process has exited, normalExit={}, exitStatus={}", normalExit, WEXITSTATUS(status));

        // dump trace and assert trace
        if(!normalExit)
        {
          // Wait 100 ms before attempting to sit down. Otherwise, LoLA might send an invalid packet.
          usleep(100000);

          sitDown(false);

          // Dump to file first, because writing to stderr may fail for various reasons.
          Assert::logDump(false, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);
          Assert::logDump(true, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);
        }

        // quit
        exit(WIFEXITED(status) && normalExit ? WEXITSTATUS(status) : EXIT_FAILURE);
      }
    }

    // if we get to here, this is the child process    
    Thread::nameCurrentThread("Main"); // for text logging output in child (if any)
    BH_TRACE_INIT("main");

    // Acquire static data, e.g. about types
    FunctionList::execute();

    Settings settings(SystemCall::getHostName(), getBodyName(getBodyId()), Constants::naoMotionCycleTime, Settings::nao);
    if(settings.playerNumber < 0 || settings.bodyName.empty())
      return EXIT_FAILURE;


    TextLogging::logStartupTime(tlogger, TextLogging::INFO);


    if(settings.headName == settings.bodyName)
      TLOGI(tlogger, "Hi, I am %s.", settings.headName.c_str());
    else
      TLOGI(tlogger, "Hi, I am {} (using {}'s body).", settings.headName.c_str(), settings.bodyName.c_str());
    TLOGI(tlogger, "teamNumber {}", settings.teamNumber);
    TLOGI(tlogger, "fieldPlayerColor {}", TypeRegistry::getEnumName(settings.fieldPlayerColor));
    TLOGI(tlogger, "goalkeeperColor {}", TypeRegistry::getEnumName(settings.goalkeeperColor));
    TLOGI(tlogger, "playerNumber {}", settings.playerNumber);
    TLOGI(tlogger, "location {}", settings.location.c_str());
    TLOGI(tlogger, "scenario {}", settings.scenario.c_str());
    TLOGI(tlogger, "magicNumber {}", settings.magicNumber);

    // put the current body name in a file (at the moment only used in downloadCalibration)
    OutTextRawFile("/var/volatile/tmp/bodyName.txt") << settings.bodyName;

    // register signal handler for ctrl+c and termination signal - for graceful shutdown
    signal(SIGTERM, sighandlerShutdown);
    signal(SIGINT, sighandlerShutdown);

    // start asynchronous logging so as not to block any of the main robot threads on I/O
    // TextLogging::requestAsyncOutput(true); // TODO

    roboeireannStart(settings);

    // settings go out of scope here, but everything that needs it later makes a copy.
  }

  while(run)
    pause();

  roboeireannStop();
  sitDown(true);

  TextLogging::requestAsyncOutput(false);
  TLOGI(tlogger, "RoboEireann stopped.");
  fflush(NULL);

  if(shutdownNAO)
  {
    TLOGI(tlogger, "About to power off the Nao");
    fflush(NULL);

    Thread::sleep(500);

    // to be sure to be sure...
    TLOGI(tlogger, "power off");
    fflush(NULL);

    static_cast<void>(!system("sudo systemctl poweroff &"));
  }

  return EXIT_SUCCESS;
}
