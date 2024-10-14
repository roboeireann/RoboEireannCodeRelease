/**
 * @file Platform/Nao/Main.cpp
 * Implementation of the main() function for starting and stopping the module framework.
 * @author Colin Graf
 */

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <sys/file.h> // flock
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "Tools/Framework/Robot.h"
#include "Tools/RobotParts/Joints.h"
#include "Platform/File.h"
#include "Tools/Communication/MsgPack.h"
#include "Tools/FunctionList.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Constants.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"

#include "Tools/TextLogging.h"

static pid_t roboeireannPid = 0;
static Robot* robot = nullptr;
static bool run = true;
static pthread_t mainThread;
static bool shutdownNAO = false;

DECL_TLOGGER(tlogger, "Main", TextLogging::INFO);

static void roboeireannStart(const Settings& settings)
{
  TLOGI(tlogger, "Roboeireann: Start.");

  robot = new Robot(settings, std::string());
  robot->start();
}

static void roboeireannStop()
{
  TLOGI(tlogger, "Roboeireann: Stop.");

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
    TLOGI(tlogger, "Caught termination/interrupt signal ({}) on non-main thread -- redirecting to main thread...",
          Assert::getSignalName(sig));

    if (sig == SIGUSR1)
    {
      TLOGI(tlogger, "Signal is sigUSR1 so prepare to power off the Nao...", Assert::getSignalName(sig));
      shutdownNAO = true;
    }

    pthread_kill(mainThread, sig);
  }
  else
  {
    if(run)
    {
      TLOGI(tlogger, "Caught (normal) termination/interrupt signal ({}) -- Shutting down executable...", Assert::getSignalName(sig));
      fflush(NULL); // flush all streams
    }
    run = false;
  }
}

static void sighandlerHardShutdown(int sig)
{
  TLOGE(tlogger, "Caught unexpected signal ({}) -- Hard shutdown of executable now!...", Assert::getSignalName(sig));
  fflush(NULL); // try to flush all streams

  raise(sig); // re-raise the signal for default handling - may dump core
}

static void sighandlerRedirect(int sig)
{
  Thread::sleep(100);
  TLOGI(tlogger, "sighandlerRedirect, caught {}, set run to false", Assert::getSignalName(sig));
  fflush(NULL); // try to flush all streams
  run = false;
}

STREAMABLE(Robots,
{
  STREAMABLE(RobotId,
  {,
    (std::string) name,
    (std::string) headId,
    (std::string) bodyId,
  }),

  (std::vector<RobotId>) robotsIds,
});

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



/** write led values to LOLA packet (at offset given by p) */
static void writeLeds(unsigned char *&p, bool ok)
{
  // Switch off all leds, except for the eyes (ok: half-blue, crashed: multicoloured)
  constexpr int NUM_LED_CATEGORIES                     = 8;
  static std::string ledCategories[NUM_LED_CATEGORIES] = {"LEye",  "REye",  "LEar",  "REar",
                                                          "Skull", "Chest", "LFoot", "RFoot"};
  static size_t ledNumbers[NUM_LED_CATEGORIES] = {24, 24, 10, 10, 12, 3, 3, 3}; // numbers of leds in each category
  for (int iLedCategory = 0; iLedCategory < NUM_LED_CATEGORIES; ++iLedCategory)
  {
    MsgPack::write(ledCategories[iLedCategory], p); // category name
    MsgPack::writeArrayHeader(ledNumbers[iLedCategory], p); // array header before array values

    // switch off any leds not related to the eyes
    if (iLedCategory >= 2)
    {
      for (size_t j = 0; j < ledNumbers[iLedCategory]; ++j)
        MsgPack::write(0.f, p);
    }
    else if (ok) // eyes and shutdown normal (ctrl-c or systemctl stop)
    {
      // Blue and white
      for (size_t j = 0; j < ledNumbers[iLedCategory]; ++j)
      {
        // red leds are 0-7, green are 8-15, and blue are 16-23
        bool isBlueLedIndex = (j >= 16) && ((j % 2) == 0);
        bool isCyanIndex    = (j >= 8) && ((j % 2) == 1);
        MsgPack::write((isBlueLedIndex || isCyanIndex) ? 1.0f : 0.f, p);
      }
    }
    else // eyes and abnormal shutdown (i.e. crash)
    {
      // display a very obvious abnormal pattern on the eyes - red, yellow, blue
      // (yellow means red and green leds on)
      for (size_t j = 0; j < ledNumbers[iLedCategory]; ++j)
      {
        // red leds are 0-7, green are 8-15, and blue are 16-23
        bool isRed    = (j < 8) && ((j % 4) == 0); // half the leds are red
        bool isYellow = (j < 16) && ((j % 4) == 2);
        bool isBlue   = (j >= 16) && ((j % 2) == 1);
        MsgPack::write((isRed || isYellow || isBlue) ? 1.0f : 0.f, p);
      }
    }
  }
}


static void sitDown(bool ok)
{
  TLOGI(tlogger, "sit down the Nao if needed, normalShutdown {}", ok);

  static const Angle targetAngles[Joints::numOfJoints - 1] =
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
  if (!connect(socket, reinterpret_cast<sockaddr *>(&address), sizeof(address)))
  {
    unsigned char packet[1000]; // was 896
    long bytesReceived = recv(socket, reinterpret_cast<char*>(packet), static_cast<int>(sizeof(packet)), 0);

    // Determine current angles and whether sitting down is required, i.e. has the hip stiffness?
    bool sitDownRequired = false;
    Angle startAngles[Joints::numOfJoints - 1];
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


    // If sitting down is required, interpolate from start angles to target angles
    if (sitDownRequired)
    {
      TLOGI(tlogger, "...sit down is required");

      int ledsWritten = 0;
      float phase = 0.f;
      while (phase < 1.f)
      {
        unsigned char *p = packet;

        // update the LEDs as soon as possible during the shutdown sit down sequence 
        // so that (human) teammembers know what is happening
        if (0 == ledsWritten++)
        {
          MsgPack::writeMapHeader(9, p); // include LEDs and joints
          writeLeds(p, ok);
        }
        else
          MsgPack::writeMapHeader(1, p); // joints only

        // write joints
        MsgPack::write("Position", p);
        MsgPack::writeArrayHeader(Joints::numOfJoints - 1, p);
        // The should pitch joints interpolate faster because to avoid collisions of the arms with the legs.
        const float shoulderPitchPhase = std::sqrt(std::min(1.f, phase / 0.6f));
        for (int i = 0; i < Joints::numOfJoints - 1; ++i)
        {
          if (i == 2 || i == 18)
            MsgPack::write(targetAngles[i] * shoulderPitchPhase + startAngles[i] * (1.f - shoulderPitchPhase), p);
          else
            MsgPack::write(targetAngles[i] * phase + startAngles[i] * (1.f - phase), p);
        }

        // Send packet to LoLA
        send(socket, reinterpret_cast<char *>(packet), static_cast<int>(p - packet), 0);

        // Receive next packet (required for sending again)
        recv(socket, reinterpret_cast<char *>(packet), static_cast<int>(sizeof(packet)), 0);

        phase += Constants::motionCycleTime / 2.f; // 2 seconds
      }
    }

    // Switch off stiffness of all joints
    unsigned char *p = packet;
    MsgPack::writeMapHeader(9, p); // must include stiffness and LEDs
    MsgPack::write("Stiffness", p);
    MsgPack::writeArrayHeader(Joints::numOfJoints - 1, p);
    for (int i = 0; i < Joints::numOfJoints - 1; ++i)
      MsgPack::write(0.f, p);

    writeLeds(p, ok); // to be sure to be sure

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
      if(robot.bodyId == bodyId)
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
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    mainThread = pthread_self();
    Thread::nameCurrentThread("ParentMain"); // for text logging output

    // parse command-line arguments
    bool watchdog = false;

    for(int i = 1; i < argc; ++i)
    {
      if(!strcmp(argv[i], "-w"))
        watchdog = true;
      else
      {
        TLOGI(tlogger, "\n  Usage: {}} [-w]\n\
      -w            use a watchdog for crash recovery and creating trace dumps", argv[0]);
        exit(EXIT_FAILURE);
      }
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
    if(chdir("/home/nao") != 0)
    {
      TLOGE(tlogger, "chdir to config directory failed!");
      exit(EXIT_FAILURE);
    }

    // the watchdog
    if(watchdog)
    {
      for(;;)
      {
        roboeireannPid = fork();
        if (roboeireannPid == -1)
          exit(EXIT_FAILURE);
        else if (roboeireannPid == 0)
          break;

        // if we get this far we are the parent (watchdog) version of the process
        int status;
        signal(SIGTERM, sighandlerRedirect);
        signal(SIGINT, sighandlerRedirect);
        if (waitpid(roboeireannPid, &status, 0) != roboeireannPid)
        {
          exit(EXIT_FAILURE);
        }
        signal(SIGTERM, SIG_DFL);
        signal(SIGINT, SIG_DFL);

        // detect requested or normal exit
        bool normalExit = !run || (WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);

        TLOGI(tlogger, "Child process has exited, normalExit={}, exitStatus={}", normalExit, WEXITSTATUS(status));

        // dump trace and assert trace
        if (!normalExit)
        {
          // Wait 100 ms before attempting to sitdown. Otherwise, LoLA might send an invalid packet.
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

    Thread::nameCurrentThread("Main"); // for text logging output in child (if any)
    BH_TRACE_INIT("main");

    // Acquire static data, e.g. about types
    FunctionList::execute();

    Settings settings(SystemCall::getHostName(), getBodyName(getBodyId()));
    if(settings.playerNumber < 0 || settings.bodyName.empty())
      return EXIT_FAILURE;

    // print status information
    char timebuf[22];
    time_t timeNow = time(NULL);
    struct tm * tmNow = localtime(&timeNow);
    TLOG_VERIFY_ERRNO(tlogger, tmNow != nullptr);
    TLOG_VERIFY(tlogger, strftime(timebuf, sizeof(timebuf), "%F %H:%M:%S", tmNow) > 0);

    TLOGI(tlogger, "----------------------------------------------------------");
    TLOGI(tlogger, ">>>>>      Startup time is: {}       <<<<<", timebuf);
    TLOGI(tlogger, "----------------------------------------------------------");

    if(settings.headName == settings.bodyName)
      TLOGI(tlogger, "Hi, I am {}.", settings.headName);
    else
      TLOGI(tlogger, "Hi, I am {} (using {}'s body).", settings.headName, settings.bodyName);
    TLOGI(tlogger, "teamNumber {}", settings.teamNumber);
    TLOGI(tlogger, "teamPort {}", settings.teamPort);
    TLOGI(tlogger, "fieldPlayerColor {}", TypeRegistry::getEnumName(settings.fieldPlayerColor));
    TLOGI(tlogger, "goalkeeperColor {}", TypeRegistry::getEnumName(settings.goalkeeperColor));
    TLOGI(tlogger, "playerNumber {}", settings.playerNumber);
    TLOGI(tlogger, "location {}", settings.location.c_str());
    TLOGI(tlogger, "scenario {}", settings.scenario.c_str());
    TLOGI(tlogger, "magicNumber {}", settings.magicNumber);

    // register signal handler for ctrl+c and termination signal and watchdog timeout
    struct sigaction act;
    memset(&act, 0, sizeof(act));
    // block all other signals while the handler is running
    sigfillset(&act.sa_mask);

    // graceful shutdown
    act.sa_handler = sighandlerShutdown;
    sigaction(SIGINT,  &act, NULL);
    sigaction(SIGTERM, &act, NULL);
    sigaction(SIGUSR1, &act, NULL);

    // reset the handler to default as soon as we enter the signal handler
    // (This is only useful for signals which should dump core)
    act.sa_flags = SA_RESETHAND;

    // hard (non-graceful) shutdown with possible core dump
    act.sa_handler = sighandlerHardShutdown;
    sigaction(SIGALRM,  &act, NULL);
    sigaction(SIGABRT,  &act, NULL);
    sigaction(SIGSEGV,  &act, NULL);
    sigaction(SIGFPE,  &act, NULL);
    sigaction(SIGBUS,  &act, NULL);

    roboeireannStart(settings);

    // settings go out of scope here, but everything that needs it later makes a copy.
  }

  while(run)
    pause();

  roboeireannStop();
  sitDown(true);
  if (shutdownNAO)
  {
    TLOGI(tlogger, "Powering off the Nao now");
    fflush(NULL);

    Thread::sleep(500);

    static_cast<void>(!system("sudo systemctl poweroff &"));
  }

  return EXIT_SUCCESS;
}
