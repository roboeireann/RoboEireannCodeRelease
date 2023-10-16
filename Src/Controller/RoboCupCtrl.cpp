/**
 * @file Controller/RoboCupCtrl.cpp
 *
 * This file implements the class RoboCupCtrl.
 *
 * @author Thomas Röfer
 * @author <A href="mailto:kspiess@tzi.de">Kai Spiess</A>
 * @author Colin Graf
 */

#include "RoboCupCtrl.h"
#include "ControllerRobot.h"
#include "RobotConsole.h"
#include "Platform/Time.h"
#include "Tools/Settings.h"

#include "Tools/TextLogging.h"

#include <QApplication>
#include <QIcon>

#ifdef MACOS
#include "Controller/Visualization/Helper.h"
#define TOLERANCE (3.f * simStepLength)
#else
#define TOLERANCE simStepLength
#endif



RoboCupCtrl* RoboCupCtrl::controller = nullptr;
SimRobot::Application* RoboCupCtrl::application = nullptr;

DECL_TLOGGER(tlogger, "RoboCupCtrl", TextLogging::INFO);


RoboCupCtrl::RoboCupCtrl(SimRobot::Application& application)
{
  Thread::nameCurrentThread("Main");
  TLOGI(tlogger, "thread has been named as Main");

  controller = this;
  RoboCupCtrl::application = &application;
  Q_INIT_RESOURCE(Controller);
}

bool RoboCupCtrl::compile()
{
  static_assert(static_cast<int>(SimRobotCore2::scene) != static_cast<int>(SimRobotCore2D::scene),
                "The kinds 'scene' must be different to distinguish between simulation cores.");

  // find simulation object
  SimRobotCore2D::Scene* scene2D = nullptr;
  SimRobotCore2::Scene* scene = static_cast<SimRobotCore2::Scene*>(application->resolveObject("RoboCup", SimRobotCore2::scene));
  if(!scene)
  {
    scene2D = static_cast<SimRobotCore2D::Scene*>(application->resolveObject("RoboCup", SimRobotCore2D::scene));
    if(!scene2D)
      return false;
    is2D = true;
  }
  else
    is2D = false;

  // Get colors of first and second team
  // we'll accept teamColors or fieldPlayerColors to refer to the outfield player colours
  // If there is no team colors compound, fall back to default
  SimRobot::Object *teamColors =
      application->resolveObject("RoboCup.teamColors", is2D ? static_cast<int>(SimRobotCore2D::compound)
                                                            : static_cast<int>(SimRobotCore2::compound));
  if (teamColors == nullptr)
    teamColors =
        application->resolveObject("RoboCup.fieldPlayerColors", is2D ? static_cast<int>(SimRobotCore2D::compound)
                                                                     : static_cast<int>(SimRobotCore2::compound));
  if (teamColors != nullptr)
  {
    ASSERT(application->getObjectChildCount(*teamColors) == 2);
    const QString& fullNameFirstTeamColor = application->getObjectChild(*teamColors, 0u)->getFullName();
    const QString& fullNameSecondTeamColor = application->getObjectChild(*teamColors, 1u)->getFullName();
    const std::string baseNameFirstTeamColor = fullNameFirstTeamColor.mid(fullNameFirstTeamColor.lastIndexOf('.') + 1).toUtf8().constData();
    const std::string baseNameSecondTeamColor = fullNameSecondTeamColor.mid(fullNameSecondTeamColor.lastIndexOf('.') + 1).toUtf8().constData();

    firstTeamFieldPlayerColor = static_cast<Settings::TeamColor>(TypeRegistry::getEnumValue(typeid(Settings::TeamColor).name(), baseNameFirstTeamColor));
    secondTeamFieldPlayerColor = static_cast<Settings::TeamColor>(TypeRegistry::getEnumValue(typeid(Settings::TeamColor).name(), baseNameSecondTeamColor));
  }

  if (firstTeamFieldPlayerColor == static_cast<Settings::TeamColor>(-1))
    firstTeamFieldPlayerColor = Settings::TeamColor::green;
  if (secondTeamFieldPlayerColor == static_cast<Settings::TeamColor>(-1))
    secondTeamFieldPlayerColor = Settings::TeamColor::red;

  SimRobot::Object *goalkeeperColors =
      application->resolveObject("RoboCup.goalkeeperColors", is2D ? static_cast<int>(SimRobotCore2D::compound)
                                                            : static_cast<int>(SimRobotCore2::compound));

  if (goalkeeperColors != nullptr)
  {
    ASSERT(application->getObjectChildCount(*goalkeeperColors) == 2);
    const QString& fullNameColor1 = application->getObjectChild(*goalkeeperColors, 0u)->getFullName();
    const QString& fullNameColor2 = application->getObjectChild(*goalkeeperColors, 1u)->getFullName();
    const std::string baseNameColor1 = fullNameColor1.mid(fullNameColor1.lastIndexOf('.') + 1).toUtf8().constData();
    const std::string baseNameColor2 = fullNameColor2.mid(fullNameColor2.lastIndexOf('.') + 1).toUtf8().constData();

    firstTeamGoalkeeperColor = static_cast<Settings::TeamColor>(TypeRegistry::getEnumValue(typeid(Settings::TeamColor).name(), baseNameColor1));
    secondTeamGoalkeeperColor = static_cast<Settings::TeamColor>(TypeRegistry::getEnumValue(typeid(Settings::TeamColor).name(), baseNameColor2));
  }

  if (firstTeamGoalkeeperColor == static_cast<Settings::TeamColor>(-1))
    firstTeamGoalkeeperColor = firstTeamFieldPlayerColor;
  if (secondTeamGoalkeeperColor == static_cast<Settings::TeamColor>(-1))
    secondTeamGoalkeeperColor = secondTeamFieldPlayerColor;

  gameController.setTeamInfos(firstTeamFieldPlayerColor, firstTeamGoalkeeperColor, 
                              secondTeamFieldPlayerColor, secondTeamGoalkeeperColor, 
                              Settings::getDefaultPortForTeam(1), Settings::getDefaultPortForTeam(2));

  // initialize simulated time and step length
  Time::initialize();
  simStepLength = static_cast<float>(is2D ? scene2D->getStepLength() : scene->getStepLength()) * 1000.f;
  delayTime = simStepLength;

  // get interfaces to simulated objects
  SimRobot::Object* group = application->resolveObject("RoboCup.robots", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore2::compound));

  for(unsigned currentRobot = 0, count = application->getObjectChildCount(*group); currentRobot < count; ++currentRobot)
  {
    SimRobot::Object* robot = application->getObjectChild(*group, currentRobot);
    const QString& fullName = robot->getFullName();
    const bool firstTeam = SimulatedRobot::isFirstTeam(robot);
    robots.push_back(new ControllerRobot(Settings("Nao", "Nao",
                                                  firstTeam ? 1 : 2,
                                                  firstTeam ? firstTeamFieldPlayerColor : secondTeamFieldPlayerColor,
                                                  firstTeam ? firstTeamGoalkeeperColor : secondTeamGoalkeeperColor,
                                                  RobotNumber::getPlayerNumberFromScene(SimulatedRobot::getSceneNumber(robot)),
                                                  location, scenarios[firstTeam ? 0 : 1],
                                                  -1, 0),
                                         fullName.mid(fullName.lastIndexOf('.') + 1).toUtf8().constData()));
  }

  const SimRobot::Object* balls = application->resolveObject("RoboCup.balls", is2D ? static_cast<int>(SimRobotCore2D::compound) : static_cast<int>(SimRobotCore2::compound));
  if(balls)
  {
    SimulatedRobot::setBall(application->getObjectChild(*balls, 0));
    if(is2D)
    {
      SimRobotCore2D::Geometry* ballGeom = static_cast<SimRobotCore2D::Geometry*>(application->resolveObject("RoboCup.balls.ball.DiskGeometry", SimRobotCore2D::geometry));
      if(ballGeom)
        ballGeom->registerCollisionCallback(*this);
      ballGeometry = ballGeom;
    }
    else
    {
      SimRobotCore2::Geometry* ballGeom = static_cast<SimRobotCore2::Geometry*>(application->resolveObject("RoboCup.balls.ball.SphereGeometry", SimRobotCore2::geometry));
      if(ballGeom)
        ballGeom->registerCollisionCallback(*this);
      ballGeometry = ballGeom;
    }
  }

  return true;
}

RoboCupCtrl::~RoboCupCtrl()
{
  qDeleteAll(views);
  SimulatedRobot::setBall(nullptr);
  Time::deinitialize();
  controller = nullptr;
  application = nullptr;
}

QBrush RoboCupCtrl::getAlternateBackgroundColor() const
{
#ifdef MACOS
  return getAlternateBase();
#else
  return QApplication::palette().alternateBase();
#endif
}

void RoboCupCtrl::addView(SimRobot::Object* object, const SimRobot::Object* parent, int flags)
{
  views.append(object);
  application->registerObject(*this, *object, parent, flags | SimRobot::Flag::showParent);
}

void RoboCupCtrl::addView(SimRobot::Object* object, const QString& categoryName, int flags)
{
  SimRobot::Object* category = application->resolveObject(categoryName);
  if(!category)
  {
    int lio = categoryName.lastIndexOf('.');
    QString subParentName = categoryName.mid(0, lio);
    QString name = categoryName.mid(lio + 1);
    category = addCategory(name, subParentName);
  }
  addView(object, category, flags);
}

void RoboCupCtrl::removeView(SimRobot::Object* object)
{
  views.removeOne(object);
  application->unregisterObject(*object);
}

void RoboCupCtrl::removeCategory(SimRobot::Object* object)
{
  views.removeOne(object);
  application->unregisterObject(*object);
}

SimRobot::Object* RoboCupCtrl::addCategory(const QString& name, const SimRobot::Object* parent, const char* icon)
{
  class Category : public SimRobot::Object
  {
    QString name;
    QString fullName;
    QIcon icon;

  public:
    Category(const QString& name, const QString& fullName, const char* icon) : name(name), fullName(fullName), icon(icon) {}

  private:
    const QString& getFullName() const override { return fullName; }
    const QIcon* getIcon() const override { return &icon; }
  };

  SimRobot::Object* category = new Category(name, parent ? parent->getFullName() + "." + name : name, icon ? icon : ":/Icons/folder.png");
  views.append(category);
  application->registerObject(*this, *category, parent, SimRobot::Flag::windowless | SimRobot::Flag::hidden);
  return category;
}

SimRobot::Object* RoboCupCtrl::addCategory(const QString& name, const QString& parentName)
{
  SimRobot::Object* parent = application->resolveObject(parentName);
  if(!parent)
  {
    int lio = parentName.lastIndexOf('.');
    QString subParentName = parentName.mid(0, lio);
    QString name = parentName.mid(lio + 1);
    parent = addCategory(name, subParentName);
  }
  return addCategory(name, parent);
}

void RoboCupCtrl::start()
{
#ifdef WINDOWS
  VERIFY(timeBeginPeriod(1) == TIMERR_NOERROR);
#endif
  DebugSenderBase::terminating = false;
  for(ControllerRobot* robot : robots)
    robot->start();
}

void RoboCupCtrl::stop()
{
  DebugSenderBase::terminating = true;
  for(ControllerRobot* robot : robots)
    robot->announceStop();
  for(ControllerRobot* robot : robots)
  {
    robot->stop();
    delete robot;
  }
  controller = nullptr;
#ifdef WINDOWS
  VERIFY(timeEndPeriod(1) == TIMERR_NOERROR);
#endif
}

void RoboCupCtrl::update()
{
  if(delayTime != 0.f)
  {
    float t = static_cast<float>(Time::getRealSystemTime());
    lastTime += delayTime;
    if(lastTime > t) // simulation is running faster then rt
    {
      if(lastTime > t + TOLERANCE)
        Thread::sleep(int(lastTime - t - TOLERANCE));
    }
    else if(t > lastTime + TOLERANCE) // slower then rt
      lastTime = t - TOLERANCE;
  }

  gameController.updateAndReferee();

  statusText = "";
  for(ControllerRobot* robot : robots)
    robot->update();
  Time::addSimulatedTime(static_cast<int>(simStepLength + 0.5f));
}

void RoboCupCtrl::collided(SimRobotCore2::Geometry&, SimRobotCore2::Geometry& geom2)
{
  SimRobotCore2::Body* body = geom2.getParentBody();
  if(!body)
    return;
  body = body->getRootBody();
  controller->gameController.setLastBallContactRobot(body);
}

void RoboCupCtrl::collided(SimRobotCore2D::Geometry&, SimRobotCore2D::Geometry& geom2)
{
  SimRobotCore2D::Body* body = geom2.getParentBody();
  if(!body)
    return;
  body = body->getRootBody();
  controller->gameController.setLastBallContactRobot(body);
}
