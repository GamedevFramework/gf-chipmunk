#include <limits>

#include <gf/Action.h>
#include <gf/Clock.h>
#include <gf/Color.h>
#include <gf/EntityContainer.h>
#include <gf/Event.h>
#include <gf/ModelContainer.h>
#include <gf/Random.h>
#include <gf/RenderWindow.h>
#include <gf/Shapes.h>
#include <gf/ViewContainer.h>
#include <gf/Views.h>
#include <gf/Window.h>

#include <gfcp/Physics.h>

namespace {

  constexpr int ImageWidth = 188;
  constexpr int ImageHeight = 35;
  constexpr int ImageRowLength = 24;

  constexpr char ImageBitmap[] = {
    15,-16,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,-64,15,63,-32,-2,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,31,-64,15,127,-125,-1,-128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,127,-64,15,127,15,-1,-64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,-1,-64,15,-2,
    31,-1,-64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,-1,-64,0,-4,63,-1,-32,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,1,-1,-64,15,-8,127,-1,-32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,-1,-64,0,-8,-15,-1,-32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,-31,-1,-64,15,-8,-32,
    -1,-32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,-15,-1,-64,9,-15,-32,-1,-32,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,31,-15,-1,-64,0,-15,-32,-1,-32,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,63,-7,-1,-64,9,-29,-32,127,-61,-16,63,15,-61,-1,-8,31,-16,15,-8,126,7,-31,
    -8,31,-65,-7,-1,-64,9,-29,-32,0,7,-8,127,-97,-25,-1,-2,63,-8,31,-4,-1,15,-13,
    -4,63,-1,-3,-1,-64,9,-29,-32,0,7,-8,127,-97,-25,-1,-2,63,-8,31,-4,-1,15,-13,
    -2,63,-1,-3,-1,-64,9,-29,-32,0,7,-8,127,-97,-25,-1,-1,63,-4,63,-4,-1,15,-13,
    -2,63,-33,-1,-1,-32,9,-25,-32,0,7,-8,127,-97,-25,-1,-1,63,-4,63,-4,-1,15,-13,
    -1,63,-33,-1,-1,-16,9,-25,-32,0,7,-8,127,-97,-25,-1,-1,63,-4,63,-4,-1,15,-13,
    -1,63,-49,-1,-1,-8,9,-57,-32,0,7,-8,127,-97,-25,-8,-1,63,-2,127,-4,-1,15,-13,
    -1,-65,-49,-1,-1,-4,9,-57,-32,0,7,-8,127,-97,-25,-8,-1,63,-2,127,-4,-1,15,-13,
    -1,-65,-57,-1,-1,-2,9,-57,-32,0,7,-8,127,-97,-25,-8,-1,63,-2,127,-4,-1,15,-13,
    -1,-1,-57,-1,-1,-1,9,-57,-32,0,7,-1,-1,-97,-25,-8,-1,63,-1,-1,-4,-1,15,-13,-1,
    -1,-61,-1,-1,-1,-119,-57,-32,0,7,-1,-1,-97,-25,-8,-1,63,-1,-1,-4,-1,15,-13,-1,
    -1,-61,-1,-1,-1,-55,-49,-32,0,7,-1,-1,-97,-25,-8,-1,63,-1,-1,-4,-1,15,-13,-1,
    -1,-63,-1,-1,-1,-23,-49,-32,127,-57,-1,-1,-97,-25,-1,-1,63,-1,-1,-4,-1,15,-13,
    -1,-1,-63,-1,-1,-1,-16,-49,-32,-1,-25,-1,-1,-97,-25,-1,-1,63,-33,-5,-4,-1,15,
    -13,-1,-1,-64,-1,-9,-1,-7,-49,-32,-1,-25,-8,127,-97,-25,-1,-1,63,-33,-5,-4,-1,
    15,-13,-1,-1,-64,-1,-13,-1,-32,-49,-32,-1,-25,-8,127,-97,-25,-1,-2,63,-49,-13,
    -4,-1,15,-13,-1,-1,-64,127,-7,-1,-119,-17,-15,-1,-25,-8,127,-97,-25,-1,-2,63,
    -49,-13,-4,-1,15,-13,-3,-1,-64,127,-8,-2,15,-17,-1,-1,-25,-8,127,-97,-25,-1,
    -8,63,-49,-13,-4,-1,15,-13,-3,-1,-64,63,-4,120,0,-17,-1,-1,-25,-8,127,-97,-25,
    -8,0,63,-57,-29,-4,-1,15,-13,-4,-1,-64,63,-4,0,15,-17,-1,-1,-25,-8,127,-97,
    -25,-8,0,63,-57,-29,-4,-1,-1,-13,-4,-1,-64,31,-2,0,0,103,-1,-1,-57,-8,127,-97,
    -25,-8,0,63,-57,-29,-4,-1,-1,-13,-4,127,-64,31,-2,0,15,103,-1,-1,-57,-8,127,
    -97,-25,-8,0,63,-61,-61,-4,127,-1,-29,-4,127,-64,15,-8,0,0,55,-1,-1,-121,-8,
    127,-97,-25,-8,0,63,-61,-61,-4,127,-1,-29,-4,63,-64,15,-32,0,0,23,-1,-2,3,-16,
    63,15,-61,-16,0,31,-127,-127,-8,31,-1,-127,-8,31,-128,7,-128,0,0
  };

  int getPixel(int x, int y) {
    return (ImageBitmap[(x >> 3) + y * ImageRowLength] >> (~x & 0x07)) & 1;
  }

  gfcp::Shape makeBall(float x, float y) {
    gfcp::Body body(1.0f, std::numeric_limits<float>::infinity());
    body.setPosition({ x, y });

    gfcp::CircleShape shape(body, 0.95f, { 0.0f, 0.0f });
    shape.setElasticity(0.0f);
    shape.setFriction(0.0f);

    return shape;
  }

  gfcp::Space makeSpace(gf::Random& random) {
    gfcp::Space space;
//     space.setIterations(1);
    space.useSpatialHash(2.0, 10000);

    for (int y = 0; y < ImageHeight; ++y) {
      for (int x = 0; x < ImageWidth; ++x) {
        if (getPixel(x, y) == 0) {
          continue;
        }

        float xJitter = random.computeUniformFloat(0.0f, 0.05f);
        float yJitter = random.computeUniformFloat(0.0f, 0.05f);

        auto shape = makeBall(2 * (x - ImageWidth / 2 + xJitter), 2 * (y - ImageHeight / 2 + yJitter));
        space.addBody(shape.getBody());
        space.addShape(shape);
      }
    }

    gfcp::Body body(1e9f, std::numeric_limits<float>::infinity());
    body.setPosition({ -1000.0f, -10.0f });
    body.setVelocity({ 400.0f, 0.0f });
    space.addBody(body);

    gfcp::CircleShape shape(body, 10.0f, { 0.0f, 0.0f });
    shape.setElasticity(0.0f);
    shape.setFriction(0.0f);
    space.addShape(shape);

    return space;
  }


}



int main() {
  static constexpr gf::Vector2i ScreenSize(1280, 720);
  static constexpr gf::Vector2f ViewSize(1024, 768);
  static constexpr gf::Vector2f ViewCenter(0.0f, 0.0f);

  // initialization

  gf::Window window("gf-chipmunk", ScreenSize);
  window.setVerticalSyncEnabled(true);
  window.setFramerateLimit(60);

  gf::RenderWindow renderer(window);

  // views

  gf::ViewContainer views;

  gf::ExtendView mainView(ViewCenter, ViewSize);
  views.addView(mainView);

  views.setInitialFramebufferSize(ScreenSize);

  // actions

  gf::ActionContainer actions;

  gf::Action closeWindowAction("Close window");
  closeWindowAction.addCloseControl();
  closeWindowAction.addKeycodeKeyControl(gf::Keycode::Escape);
  actions.addAction(closeWindowAction);

  gf::Action fullscreenAction("Fullscreen");
  fullscreenAction.addKeycodeKeyControl(gf::Keycode::F);
  actions.addAction(fullscreenAction);

  // models

  gf::ModelContainer models;

  gf::Random random;
  auto space = makeSpace(random);
  models.addModel(space);

  // entities

  gf::EntityContainer mainEntities;

//   gfb2d::PhysicsDebugger debug(physics);
//   debug.setDebug(true);
//   mainEntities.addEntity(debug);

  // game loop

  renderer.clear(gf::Color::Black);

  gf::Clock clock;

  while (window.isOpen()) {
    // 1. input

    gf::Event event;

    while (window.pollEvent(event)) {
      actions.processEvent(event);
      views.processEvent(event);
    }

    if (closeWindowAction.isActive()) {
      window.close();
    }

    if (fullscreenAction.isActive()) {
      window.toggleFullscreen();
    }


    // 2. update

    gf::Time time = clock.restart();
    models.update(time);
    mainEntities.update(time);

    // 3. draw

    renderer.clear();

    renderer.setView(mainView);

    space.eachBody([&](gfcp::Body body) {
      gf::CircleShape shape(1.5f);
      shape.setPosition(body.getPosition());
      shape.setColor(gf::Color::White);
      renderer.draw(shape);
    });

    mainEntities.render(renderer);

    renderer.display();

    actions.reset();
  }

  return 0;


}
