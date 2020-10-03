#include <gfcp/Physics.h>

#include <chipmunk/chipmunk.h>

#include <gf/Color.h>

namespace gfcp {
  /*
   * Arbiter
   */

  float Arbiter::getRestitution() const {
    return cpArbiterGetRestitution(m_obj);
  }

  void Arbiter::setRestitution(float restitution) {
    cpArbiterSetRestitution(m_obj, restitution);
  }

  float Arbiter::getFriction() const {
    return cpArbiterGetFriction(m_obj);
  }

  void Arbiter::setFriction(float friction) {
    cpArbiterSetFriction(m_obj, friction);
  }

  gf::Vector2f Arbiter::getSurfaceVelocity() {
    auto velocity = cpArbiterGetSurfaceVelocity(m_obj);
    return gf::vec(velocity.x, velocity.y);
  }

  void Arbiter::setSurfaceVelocity(gf::Vector2f velocity) {
    cpArbiterSetSurfaceVelocity(m_obj, cpv(velocity.x, velocity.y));
  }

  gf::Vector2f Arbiter::computeTotalImpulse() const {
    auto impulse = cpArbiterTotalImpulse(m_obj);
    return gf::vec(impulse.x, impulse.y);
  }

  float Arbiter::computeTotalKineticEnergy() const {
    return cpArbiterTotalKE(m_obj);
  }

  bool Arbiter::ignore() {
    return cpArbiterIgnore(m_obj) == cpTrue;
  }

  std::pair<Shape, Shape> Arbiter::getShapes() const {
    cpShape * a;
    cpShape * b;
    cpArbiterGetShapes(m_obj, &a, &b);
    return std::make_pair(Shape(a), Shape(b));
  }

  std::pair<Body, Body> Arbiter::getBodies() const {
    cpBody * a;
    cpBody * b;
    cpArbiterGetBodies(m_obj, &a, &b);
    return std::make_pair(Body(a), Body(b));
  }

  bool Arbiter::isFirstContact() const {
    return cpArbiterIsFirstContact(m_obj) == cpTrue;
  }

  bool Arbiter::isRemoval() const {
    return cpArbiterIsRemoval(m_obj) == cpTrue;
  }

  int Arbiter::getCount() const {
    return cpArbiterGetCount(m_obj);
  }

  gf::Vector2f Arbiter::getNormal() const {
    auto normal = cpArbiterGetNormal(m_obj);
    return gf::vec(normal.x, normal.y);
  }

  gf::Vector2f Arbiter::getPointA(int i) const {
    auto a = cpArbiterGetPointA(m_obj, i);
    return gf::vec(a.x, a.y);
  }

  gf::Vector2f Arbiter::getPointB(int i) const {
    auto b = cpArbiterGetPointB(m_obj, i);
    return gf::vec(b.x, b.y);
  }

  float Arbiter::getDepth(int i) const {
    return cpArbiterGetDepth(m_obj, i);
  }

  bool Arbiter::callWildcardBeginA(Space space) {
    return cpArbiterCallWildcardBeginA(m_obj, space.m_obj);
  }

  bool Arbiter::callWildcardBeginB(Space space) {
    return cpArbiterCallWildcardBeginB(m_obj, space.m_obj);
  }

  bool Arbiter::callWildcardPreSolveA(Space space) {
    return cpArbiterCallWildcardPreSolveA(m_obj, space.m_obj);
  }

  bool Arbiter::callWildcardPreSolveB(Space space) {
    return cpArbiterCallWildcardPreSolveB(m_obj, space.m_obj);
  }

  void Arbiter::callWildcardPostSolveA(Space space) {
    cpArbiterCallWildcardPostSolveA(m_obj, space.m_obj);
  }

  void Arbiter::callWildcardPostSolveB(Space space) {
    cpArbiterCallWildcardPostSolveB(m_obj, space.m_obj);
  }

  void Arbiter::callWildcardSeparateA(Space space) {
    cpArbiterCallWildcardSeparateA(m_obj, space.m_obj);
  }

  void Arbiter::callWildcardSeparateB(Space space) {
    cpArbiterCallWildcardSeparateB(m_obj, space.m_obj);
  }


  /*
   * CollisionHandler
   */

  bool CollisionHandler::begin(Arbiter arbiter, Space space) {
    return true;
  }

  bool CollisionHandler::preSolve(Arbiter arbiter, Space space) {
    return true;
  }

  void CollisionHandler::postSolve(Arbiter arbiter, Space space) {
  }

  void CollisionHandler::separate(Arbiter arbiter, Space space) {
  }


  /*
   * SpaceDebug
   */

  void SpaceDebug::drawCircle(gf::Vector2f pos, float angle, float radius, gf::Color4f outlineColor, gf::Color4f fillColor) {
  }

  void SpaceDebug::drawSegment(gf::Vector2f a, gf::Vector2f b, gf::Color4f color) {
  }

  void SpaceDebug::drawFatSegment(gf::Vector2f a, gf::Vector2f b, float radius, gf::Color4f outlineColor, gf::Color4f fillColor) {
  }

  void SpaceDebug::drawPolygon(gf::Span<const gf::Vector2f> verts, float radius, gf::Color4f outlineColor, gf::Color4f fillColor) {
  }

  void SpaceDebug::drawDot(float size, gf::Vector2f pos, gf::Color4f color) {
  }

  gf::Color4f SpaceDebug::getColorForShape(Shape shape) {
    return gf::Color::Gray();
  }

  /*
   * Space
   */

  Space::Space()
  : m_obj(cpSpaceNew())
  {
  }

  void Space::dispose() {
    cpSpaceFree(m_obj);
    m_obj = nullptr;
  }

  namespace {
    // shape

    void disposeShapePostStep(cpSpace * space, void * key, void * data) {
      auto shape = static_cast<cpShape*>(key);
      cpSpaceRemoveShape(space, shape);
      cpShapeFree(shape);
    }

    void disposeShapeIterator(cpShape * shape, void * data) {
      auto space = static_cast<cpSpace*>(data);
      cpSpaceAddPostStepCallback(space, disposeShapePostStep, static_cast<void*>(shape), nullptr);
    }

    // constraints

    void disposeConstraintPostStep(cpSpace * space, void * key, void * data) {
      auto constraint = static_cast<cpConstraint*>(key);
      cpSpaceRemoveConstraint(space, constraint);
      cpConstraintFree(constraint);
    }

    void disposeConstraintIterator(cpConstraint * constraint, void * data) {
      auto space = static_cast<cpSpace*>(data);
      cpSpaceAddPostStepCallback(space, disposeConstraintPostStep, static_cast<void*>(constraint), nullptr);
    }

    // body

    void disposeBodyPostStep(cpSpace * space, void * key, void * data) {
      auto body = static_cast<cpBody*>(key);
      cpSpaceRemoveBody(space, body);
      cpBodyFree(body);
    }

    void disposeBodyIterator(cpBody * body, void * data) {
      auto space = static_cast<cpSpace*>(data);
      cpSpaceAddPostStepCallback(space, disposeBodyPostStep, static_cast<void*>(body), nullptr);
    }
  }

  void Space::disposeChildren() {
    auto space = static_cast<cpSpace*>(m_obj);
    cpSpaceEachShape(space, disposeShapeIterator, static_cast<void*>(space));
    cpSpaceEachConstraint(space, disposeConstraintIterator, static_cast<void*>(space));
    cpSpaceEachBody(space, disposeBodyIterator, static_cast<void*>(space));
  }

  int Space::getIterations() const {
    return cpSpaceGetIterations(m_obj);
  }

  void Space::setIterations(int iterations) {
    cpSpaceSetIterations(m_obj, iterations);
  }

  gf::Vector2f Space::getGravity() const {
    auto gravity = cpSpaceGetGravity(m_obj);
    return gf::vec(gravity.x, gravity.y);
  }

  void Space::setGravity(gf::Vector2f gravity) {
    cpSpaceSetGravity(m_obj, cpv(gravity.x, gravity.y));
  }

  float Space::getDamping() const {
    return cpSpaceGetDamping(m_obj);
  }

  void Space::setDamping(float damping) {
    cpSpaceSetDamping(m_obj, damping);
  }

  float Space::getIdleSpeedThreshold() const {
    return cpSpaceGetIdleSpeedThreshold(m_obj);
  }

  void Space::setIdleSpeedThreshold(float threshold) {
    cpSpaceSetIdleSpeedThreshold(m_obj, threshold);
  }

  float Space::getSleepTimeThreshold() const {
    return cpSpaceGetSleepTimeThreshold(m_obj);
  }

  void Space::setSleepTimeThreshold(float threshold) {
    cpSpaceSetSleepTimeThreshold(m_obj, threshold);
  }

  float Space::getCollisionSlop() const {
    return cpSpaceGetCollisionSlop(m_obj);
  }

  void Space::setCollisionSlop(float slop) {
    cpSpaceSetCollisionSlop(m_obj, slop);
  }

  float Space::getCollisionBias() const {
    return cpSpaceGetCollisionBias(m_obj);
  }

  void Space::setCollisionBias(float bias) {
    cpSpaceSetCollisionBias(m_obj, bias);
  }

  unsigned Space::getCollisionPersistence() const {
    return cpSpaceGetCollisionPersistence(m_obj);
  }

  void Space::setCollisionPersistence(unsigned persistence) {
    cpSpaceSetCollisionPersistence(m_obj, persistence);
  }

  float Space::getCurrentTimeStep() const {
    return cpSpaceGetCurrentTimeStep(m_obj);
  }

  bool Space::isLocked() {
    return cpSpaceIsLocked(m_obj) == cpTrue;
  }

  namespace {

    cpBool myCollisionBeginFunc(cpArbiter *arb, cpSpace *space, cpDataPointer userData) {
      auto handler = static_cast<CollisionHandler*>(userData);
      return handler->begin(Arbiter(arb), Space(space)) ? cpTrue : cpFalse;
    }

    cpBool myCollisionPreSolveFunc(cpArbiter *arb, cpSpace *space, cpDataPointer userData) {
      auto handler = static_cast<CollisionHandler*>(userData);
      return handler->preSolve(Arbiter(arb), Space(space)) ? cpTrue : cpFalse;
    }

    void myCollisionPostSolveFunc(cpArbiter *arb, cpSpace *space, cpDataPointer userData) {
      auto handler = static_cast<CollisionHandler*>(userData);
      handler->postSolve(Arbiter(arb), Space(space));
    }

    void myCollisionSeparateFunc(cpArbiter *arb, cpSpace *space, cpDataPointer userData) {
      auto handler = static_cast<CollisionHandler*>(userData);
      handler->separate(Arbiter(arb), Space(space));
    }

  }

  void Space::setDefaultCollisionHandler(CollisionHandler& handler) {
    auto raw = cpSpaceAddDefaultCollisionHandler(m_obj);
    raw->beginFunc = myCollisionBeginFunc;
    raw->preSolveFunc = myCollisionPreSolveFunc;
    raw->postSolveFunc = myCollisionPostSolveFunc;
    raw->separateFunc = myCollisionSeparateFunc;
    raw->userData = &handler;
  }

  void Space::setCollisionHandler(CollisionHandler& handler, uintptr_t a, uintptr_t b) {

  }

  void Space::setWildcardHandler(CollisionHandler& handler, uintptr_t type) {

  }

  void Space::addShape(Shape shape) {
    cpSpaceAddShape(m_obj, shape.m_obj);
  }

  void Space::addBody(Body body) {
    cpSpaceAddBody(m_obj, body.m_obj);
  }

  void Space::addConstraint(Constraint constraint) {
    cpSpaceAddConstraint(m_obj, constraint.m_obj);
  }

  void Space::removeShape(Shape shape) {
    cpSpaceRemoveShape(m_obj, shape.m_obj);
  }

  void Space::removeBody(Body body) {
    cpSpaceRemoveBody(m_obj, body.m_obj);
  }

  void Space::removeConstraint(Constraint constraint) {
    cpSpaceRemoveConstraint(m_obj, constraint.m_obj);
  }

  bool Space::containsShape(Shape shape) {
    return cpSpaceContainsShape(m_obj, shape.m_obj) == cpTrue;
  }

  bool Space::containsBody(Body body) {
    return cpSpaceContainsBody(m_obj, body.m_obj) == cpTrue;
  }

  bool Space::containsConstraint(Constraint constraint) {
    return cpSpaceContainsConstraint(m_obj, constraint.m_obj) == cpTrue;
  }

  namespace {

    void mySpaceBodyIteratorFunc(cpBody * body, void * data) {
      auto func = static_cast<std::function<void(Body)>*>(data);
      (*func)(Body(body));
    }

    void mySpaceShapeIteratorFunc(cpShape * shape, void * data) {
      auto func = static_cast<std::function<void(Shape)>*>(data);
      (*func)(Shape(shape));
    }

    void mySpaceConstraintIteratorFunc(cpConstraint *constraint, void *data) {
      auto func = static_cast<std::function<void(Constraint)>*>(data);
      (*func)(Constraint(constraint));
    }

  }

  void Space::eachBody(std::function<void(Body)> func) {
    cpSpaceEachBody(m_obj, mySpaceBodyIteratorFunc, &func);
  }

  void Space::eachShape(std::function<void(Shape)> func) {
    cpSpaceEachShape(m_obj, mySpaceShapeIteratorFunc, &func);
  }

  void Space::eachConstraint(std::function<void(Constraint)> func) {
    cpSpaceEachConstraint(m_obj, mySpaceConstraintIteratorFunc, &func);
  }

  void Space::reindexStatic() {
    cpSpaceReindexStatic(m_obj);
  }

  void Space::reindexShape(Shape& shape) {
    cpSpaceReindexShape(m_obj, shape.m_obj);
  }

  void Space::reindexShapesForBody(Body& body) {
    cpSpaceReindexShapesForBody(m_obj, body.m_obj);
  }

  void Space::useSpatialHash(float dim, int count) {
    cpSpaceUseSpatialHash(m_obj, dim, count);
  }

  namespace {

    void mySpaceDebugDrawCircle(cpVect pos, cpFloat angle, cpFloat radius, cpSpaceDebugColor oc, cpSpaceDebugColor fc, cpDataPointer data) {
      auto debug = static_cast<SpaceDebug*>(data);
      debug->drawCircle(gf::vec(pos.x, pos.y), angle, radius, gf::vec(oc.r, oc.g, oc.b, oc.a), gf::vec(fc.r, fc.g, fc.b, fc.a));
    }


    void mySpaceDebugDrawSegment(cpVect a, cpVect b, cpSpaceDebugColor color, cpDataPointer data) {
      auto debug = static_cast<SpaceDebug*>(data);
      debug->drawSegment(gf::vec(a.x, a.y), gf::vec(b.x, b.y), gf::vec(color.r, color.g, color.b, color.a));
    }


    void mySpaceDebugDrawFatSegment(cpVect a, cpVect b, cpFloat radius, cpSpaceDebugColor oc, cpSpaceDebugColor fc, cpDataPointer data) {
      auto debug = static_cast<SpaceDebug*>(data);
      debug->drawFatSegment(gf::vec(a.x, a.y), gf::vec(b.x, b.y), radius, gf::vec(oc.r, oc.g, oc.b, oc.a), gf::vec(fc.r, fc.g, fc.b, fc.a));
    }

    void mySpaceDebugDrawPolygon(int count, const cpVect *verts, cpFloat radius, cpSpaceDebugColor oc, cpSpaceDebugColor fc, cpDataPointer data) {
      std::vector<gf::Vector2f> transformed;

      for (int i = 0; i < count; ++i) {
        transformed.emplace_back(gf::vec(verts[i].x, verts[i].y));
      }

      auto debug = static_cast<SpaceDebug*>(data);
      debug->drawPolygon(transformed, radius, gf::vec(oc.r, oc.g, oc.b, oc.a), gf::vec(fc.r, fc.g, fc.b, fc.a));
    }


    void mySpaceDebugDrawDot(cpFloat size, cpVect pos, cpSpaceDebugColor color, cpDataPointer data) {
      auto debug = static_cast<SpaceDebug*>(data);
      debug->drawDot(size, gf::vec(pos.x, pos.y), gf::vec(color.r, color.g, color.b, color.a));
    }


    cpSpaceDebugColor mySpaceDebugDrawColorForShape(cpShape *shape, cpDataPointer data) {
      auto debug = static_cast<SpaceDebug*>(data);
      auto color = debug->getColorForShape(Shape(shape));
      return { color.r, color.g, color.b, color.a };
    }

  }

  void Space::debugDraw(SpaceDebug& debug) {
    cpSpaceDebugDrawOptions options;
    options.drawCircle = mySpaceDebugDrawCircle;
    options.drawSegment = mySpaceDebugDrawSegment;
    options.drawFatSegment = mySpaceDebugDrawFatSegment;
    options.drawPolygon = mySpaceDebugDrawPolygon;
    options.drawDot = mySpaceDebugDrawDot;
    options.flags = static_cast<cpSpaceDebugDrawFlags>(CP_SPACE_DEBUG_DRAW_SHAPES | CP_SPACE_DEBUG_DRAW_CONSTRAINTS | CP_SPACE_DEBUG_DRAW_COLLISION_POINTS);
    options.shapeOutlineColor = { 1.0f, 0.0f, 0.0f, 1.0f };
    options.colorForShape = mySpaceDebugDrawColorForShape;
    options.constraintColor = {  0.0f, 1.0f, 0.0f, 1.0f };
    options.collisionPointColor = { 0.0f, 0.0f, 1.0f, 1.0f };
    cpSpaceDebugDraw(m_obj, &options);
  }


  void Space::update(gf::Time time) {
    cpSpaceStep(m_obj, time.asSeconds());
  }

  /*
   * Body
   */

  static_assert(static_cast<int>(BodyType::Dynamic) == static_cast<int>(CP_BODY_TYPE_DYNAMIC), "Body type constant mismatch.");
  static_assert(static_cast<int>(BodyType::Kinematic) == static_cast<int>(CP_BODY_TYPE_KINEMATIC), "Body type constant mismatch.");
  static_assert(static_cast<int>(BodyType::Static) == static_cast<int>(CP_BODY_TYPE_STATIC), "Body type constant mismatch.");

  Body::Body(float mass, float moment)
  : m_obj(cpBodyNew(mass, moment))
  {
  }

  Body Body::makeKinematic() {
    return Body(cpBodyNewKinematic());
  }

  Body Body::makeStatic() {
    return Body(cpBodyNewStatic());
  }

  void Body::dispose() {
    cpBodyFree(m_obj);
    m_obj = nullptr;
  }

  void Body::activate() {
    cpBodyActivate(m_obj);
  }

  void Body::activateStatic(Shape filter) {
    cpBodyActivateStatic(m_obj, filter.m_obj);
  }

  void Body::sleep() {
    cpBodySleep(m_obj);
  }

  void Body::sleepWithGroup(Body group) {
    cpBodySleepWithGroup(m_obj, group.m_obj);
  }

  bool Body::isSleeping() const {
    return cpBodyIsSleeping(m_obj) == cpTrue;
  }


  BodyType Body::getType() {
    return static_cast<BodyType>(cpBodyGetType(m_obj));
  }

  void Body::setType(BodyType type) {
    cpBodySetType(m_obj, static_cast<cpBodyType>(type));
  }

  Space Body::getSpace() const {
    return Space(cpBodyGetSpace(m_obj));
  }

  float Body::getMass() const {
    return cpBodyGetMass(m_obj);
  }

  void Body::setMass(float mass) {
    cpBodySetMass(m_obj, mass);
  }

  float Body::getMoment() const {
    return cpBodyGetMoment(m_obj);
  }

  void Body::setMoment(float moment) {
    cpBodySetMoment(m_obj, moment);
  }

  gf::Vector2f Body::getPosition() const {
    auto position = cpBodyGetPosition(m_obj);
    return gf::vec(position.x, position.y);
  }

  void Body::setPosition(gf::Vector2f position) {
    cpBodySetPosition(m_obj, cpv(position.x, position.y));
  }

  gf::Vector2f Body::getCenterOfGravity() const {
    auto cog = cpBodyGetCenterOfGravity(m_obj);
    return gf::vec(cog.x, cog.y);
  }

  void Body::setCenterOfGravity(gf::Vector2f cog) {
    cpBodySetCenterOfGravity(m_obj, cpv(cog.x, cog.y));
  }

  gf::Vector2f Body::getVelocity() const {
    auto velocity = cpBodyGetVelocity(m_obj);
    return gf::vec(velocity.x, velocity.y);
  }

  void Body::setVelocity(gf::Vector2f velocity) {
    cpBodySetVelocity(m_obj, cpv(velocity.x, velocity.y));
  }

  gf::Vector2f Body::getForce() const {
    auto force = cpBodyGetForce(m_obj);
    return gf::vec(force.x, force.y);
  }

  void Body::setForce(gf::Vector2f force) {
    cpBodySetForce(m_obj, cpv(force.x, force.y));
  }

  float Body::getAngle() const {
    return cpBodyGetAngle(m_obj);
  }

  void Body::setAngle(float angle) {
    cpBodySetAngle(m_obj, angle);
  }

  float Body::getAngularVelocity() const {
    return cpBodyGetAngularVelocity(m_obj);
  }

  void Body::setAngularVelocity(float velocity) {
    cpBodySetAngularVelocity(m_obj, velocity);
  }

  float Body::getTorque() const {
    return cpBodyGetTorque(m_obj);
  }

  void Body::setTorque(float torque) {
    cpBodySetTorque(m_obj, torque);
  }

  gf::Vector2f Body::getRotation() const {
    auto rotation = cpBodyGetRotation(m_obj);
    return gf::vec(rotation.x, rotation.y);
  }

  gf::Vector2f Body::computeLocalToWorldCoordinates(gf::Vector2f point) const {
    auto coords = cpBodyLocalToWorld(m_obj, cpv(point.x, point.y));
    return gf::vec(coords.x, coords.y);
  }

  gf::Vector2f Body::computeWorldToLocalCoordinates(gf::Vector2f point) const {
    auto coords = cpBodyWorldToLocal(m_obj, cpv(point.x, point.y));
    return gf::vec(coords.x, coords.y);
  }

  void Body::applyForceAtWorldPoint(gf::Vector2f force, gf::Vector2f point) {
    cpBodyApplyForceAtWorldPoint(m_obj, cpv(force.x, force.y), cpv(point.x, point.y));
  }

  void Body::applyForceAtLocalPoint(gf::Vector2f force, gf::Vector2f point) {
    cpBodyApplyForceAtLocalPoint(m_obj, cpv(force.x, force.y), cpv(point.x, point.y));
  }

  void Body::applyImpulseAtWorldPoint(gf::Vector2f impulse, gf::Vector2f point) {
    cpBodyApplyImpulseAtWorldPoint(m_obj, cpv(impulse.x, impulse.y), cpv(point.x, point.y));
  }

  void Body::applyImpulseAtLocalPoint(gf::Vector2f impulse, gf::Vector2f point) {
    cpBodyApplyImpulseAtLocalPoint(m_obj, cpv(impulse.x, impulse.y), cpv(point.x, point.y));
  }

  gf::Vector2f Body::getVelocityAtWorldPoint(gf::Vector2f point) const {
    auto velocity = cpBodyGetVelocityAtWorldPoint(m_obj, cpv(point.x, point.y));
    return gf::vec(velocity.x, velocity.y);
  }

  gf::Vector2f Body::getVelocityAtLocalPoint(gf::Vector2f point) const {
    auto velocity = cpBodyGetVelocityAtLocalPoint(m_obj, cpv(point.x, point.y));
    return gf::vec(velocity.x, velocity.y);
  }

  float Body::getKinematicEnergy() const {
    return cpBodyKineticEnergy(m_obj);
  }

  namespace {
    void myBodyShapeIterator(cpBody * body, cpShape * shape, void * data) {
      auto func = static_cast<std::function<void(Body, Shape)>*>(data);
      (*func)(Body(body), Shape(shape));
    }

    void myBodyConstraintIteratorFunc(cpBody * body, cpConstraint * constraint, void * data) {
      auto func = static_cast<std::function<void(Body, Constraint)>*>(data);
      (*func)(Body(body), Constraint(constraint));
    }

    void myBodyArbiterIteratorFunc(cpBody * body, cpArbiter * arbiter, void * data) {
      auto func = static_cast<std::function<void(Body, Arbiter)>*>(data);
      (*func)(Body(body), Arbiter(arbiter));
    }
  }

  void Body::eachShape(std::function<void(Body, Shape)> func) {
    cpBodyEachShape(m_obj, myBodyShapeIterator, &func);
  }

  void Body::eachConstraint(std::function<void(Body, Constraint)> func) {
    cpBodyEachConstraint(m_obj, myBodyConstraintIteratorFunc, &func);
  }

  void Body::eachArbiter(std::function<void(Body, Arbiter)> func) {
    cpBodyEachArbiter(m_obj, myBodyArbiterIteratorFunc, &func);
  }

  /*
   * Shape
   */

  void Shape::dispose() {
    cpShapeFree(m_obj);
    m_obj = nullptr;
  }

  Space Shape::getSpace() const {
    return Space(cpShapeGetSpace(m_obj));
  }

  Body Shape::getBody() const {
    return Body(cpShapeGetBody(m_obj));
  }

  void Shape::setBody(Body body) {
    cpShapeSetBody(m_obj, body.m_obj);
  }

  float Shape::getMass() {
    return cpShapeGetMass(m_obj);
  }

  void Shape::setMass(float mass) {
    cpShapeSetMass(m_obj, mass);
  }

  float Shape::getDensity() {
    return cpShapeGetDensity(m_obj);
  }

  void Shape::setDensity(float density) {
    cpShapeSetDensity(m_obj, density);
  }

  float Shape::getMoment() {
    return cpShapeGetMoment(m_obj);
  }

  float Shape::getArea() {
    return cpShapeGetArea(m_obj);
  }

  gf::Vector2f Shape::getCenterOfGravity() {
    auto cog = cpShapeGetCenterOfGravity(m_obj);
    return gf::vec(cog.x, cog.y);
  }

  gf::RectF Shape::getBB() const {
    auto bb = cpShapeGetBB(m_obj);
    return gf::RectF::fromMinMax(gf::vec(bb.l, bb.b), gf::vec(bb.r, bb.t));
  }

  bool Shape::getSensor() const {
    return cpShapeGetSensor(m_obj);
  }

  void Shape::setSensor(bool sensor) {
    cpShapeSetSensor(m_obj, sensor);
  }

  float Shape::getElasticity() const {
    return cpShapeGetElasticity(m_obj);
  }

  void Shape::setElasticity(float elasticity) {
    cpShapeSetElasticity(m_obj, elasticity);
  }

  float Shape::getFriction() const {
    return cpShapeGetFriction(m_obj);
  }

  void Shape::setFriction(float friction) {
    cpShapeSetFriction(m_obj, friction);
  }

  gf::Vector2f Shape::getSurfaceVelocity() const {
    auto velocity = cpShapeGetSurfaceVelocity(m_obj);
    return gf::vec(velocity.x, velocity.y);
  }

  void Shape::setSurfaceVelocity(gf::Vector2f velocity) {
    cpShapeSetSurfaceVelocity(m_obj, cpv(velocity.x, velocity.y));
  }

  uintptr_t Shape::getCollisionType() const {
    return cpShapeGetCollisionType(m_obj);
  }

  void Shape::setCollisionType(uintptr_t type) {
    cpShapeSetCollisionType(m_obj, type);
  }

  ShapeFilter Shape::getShapeFilter() const {
    auto filter = cpShapeGetFilter(m_obj);
    return { filter.group, filter.categories, filter.mask };
  }

  void Shape::setShapeFilter(ShapeFilter filter) {
    cpShapeSetFilter(m_obj, cpShapeFilterNew(filter.group, filter.categories, filter.mask));
  }

  cpBody * Shape::unwrap(Body body) {
    return body.m_obj;
  }

  /*
   * CircleShape
   */

  CircleShape::CircleShape(Body body, float radius, gf::Vector2f offset)
  : Shape(cpCircleShapeNew(unwrap(body), radius, cpv(offset.x, offset.y)))
  {
  }

  gf::Vector2f CircleShape::getOffset() const {
    auto offset = cpCircleShapeGetOffset(m_obj);
    return gf::vec(offset.x, offset.y);
  }

  float CircleShape::getRadius() const {
    return cpCircleShapeGetRadius(m_obj);
  }

  /*
   * SegmentShape
   */

  SegmentShape::SegmentShape(Body body, gf::Vector2f a, gf::Vector2f b, float radius)
  : Shape(cpSegmentShapeNew(unwrap(body), cpv(a.x, a.y), cpv(b.x, b.y), radius))
  {
  }

  void SegmentShape::setNeighbors(gf::Vector2f prev, gf::Vector2f next) {
    cpSegmentShapeSetNeighbors(m_obj, cpv(prev.x, prev.y), cpv(next.x, next.y));
  }

  gf::Vector2f SegmentShape::getA() const {
    auto a = cpSegmentShapeGetA(m_obj);
    return gf::vec(a.x, a.y);
  }

  gf::Vector2f SegmentShape::getB() const {
    auto b = cpSegmentShapeGetB(m_obj);
    return gf::vec(b.x, b.y);
  }

  gf::Vector2f SegmentShape::getNormal() const {
    auto n = cpSegmentShapeGetNormal(m_obj);
    return gf::vec(n.x, n.y);
  }

  float SegmentShape::getRadius() const {
    return cpSegmentShapeGetRadius(m_obj);
  }

  /*
   * PolygonShape
   */

  namespace {

    cpShape* createPolygoneShape(cpBody * body, gf::Span<const gf::Vector2f> verts, gf::Matrix3f t, float radius) {
      std::vector<cpVect> transformed;

      for (auto v : verts) {
        transformed.emplace_back(cpv(v.x, v.y));
      }

      // assert on t(2, *)?

      return cpPolyShapeNew(body, static_cast<int>(transformed.size()), transformed.data(), cpTransformNew(t(0, 0), t(1, 0), t(0, 1), t(1, 1), t(0, 2), t(1, 2)), radius);
    }
  }

  PolygonShape::PolygonShape(Body body, gf::Span<const gf::Vector2f> verts, gf::Matrix3f transform, float radius)
  : Shape(createPolygoneShape(unwrap(body), verts, transform, radius))
  {
  }

  PolygonShape::PolygonShape(Body body, gf::RectF box, float radius)
  : Shape(cpBoxShapeNew2(unwrap(body), cpBBNew(box.min.x, box.min.y, box.max.x, box.max.y), radius))
  {
  }

  std::size_t PolygonShape::getPointCount() const {
    return static_cast<std::size_t>(cpPolyShapeGetCount(m_obj));
  }

  gf::Vector2f PolygonShape::getPoint(std::size_t index) const {
    auto point = cpPolyShapeGetVert(m_obj, static_cast<int>(index));
    return gf::vec(point.x, point.y);
  }

  float PolygonShape::getRadius() const {
    return cpPolyShapeGetRadius(m_obj);
  }

  /*
   * Constraint
   */

  void Constraint::dispose() {
    cpConstraintFree(m_obj);
    m_obj = nullptr;
  }

  Space Constraint::getSpace() const {
    return Space(cpConstraintGetSpace(m_obj));
  }

  Body Constraint::getBodyA() const {
    return Body(cpConstraintGetBodyA(m_obj));
  }

  Body Constraint::getBodyB() const {
    return Body(cpConstraintGetBodyB(m_obj));
  }

  float Constraint::getMaxForce() const {
    return cpConstraintGetMaxForce(m_obj);
  }

  void Constraint::setMaxForce(float force) {
    cpConstraintSetMaxForce(m_obj, force);
  }

  float Constraint::getErrorBias() const {
    return cpConstraintGetErrorBias(m_obj);
  }

  void Constraint::setErrorBias(float bias) {
    cpConstraintSetErrorBias(m_obj, bias);
  }

  float Constraint::getMaxBias() const {
    return cpConstraintGetMaxBias(m_obj);
  }

  void Constraint::setMaxBias(float bias) {
    cpConstraintSetMaxBias(m_obj, bias);
  }

  bool Constraint::getCollideBodies() const {
    return cpConstraintGetCollideBodies(m_obj) == cpTrue;
  }

  void Constraint::setCollideBodies(bool collide) {
    cpConstraintSetCollideBodies(m_obj, collide);
  }

  float Constraint::getImpulse() {
    return cpConstraintGetImpulse(m_obj);
  }

  cpBody * Constraint::unwrap(Body body) {
    return body.m_obj;
  }

  /*
   * PinJoint
   */

  PinJoint::PinJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB)
  : Constraint(cpPinJointNew(unwrap(a), unwrap(b), cpv(anchorA.x, anchorA.y), cpv(anchorB.x, anchorB.y)))
  {
  }

  gf::Vector2f PinJoint::getAnchorA() const {
    auto anchor = cpPinJointGetAnchorA(m_obj);
    return gf::vec(anchor.x, anchor.y);
  }

  void PinJoint::setAnchorA(gf::Vector2f anchor) {
    cpPinJointSetAnchorA(m_obj, cpv(anchor.x, anchor.y));
  }

  gf::Vector2f PinJoint::getAnchorB() const {
    auto anchor = cpPinJointGetAnchorB(m_obj);
    return gf::vec(anchor.x, anchor.y);
  }

  void PinJoint::setAnchorB(gf::Vector2f anchor) {
    cpPinJointSetAnchorB(m_obj, cpv(anchor.x, anchor.y));
  }

  float PinJoint::getDistance() const {
    return cpPinJointGetDist(m_obj);
  }

  void PinJoint::setDistance(float distance) {
    cpPinJointSetDist(m_obj, distance);
  }

  /*
   * SlideJoint
   */

  SlideJoint::SlideJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB, float min, float max)
  : Constraint(cpSlideJointNew(unwrap(a), unwrap(b), cpv(anchorA.x, anchorA.y), cpv(anchorB.x, anchorB.y), min, max))
  {
  }

  gf::Vector2f SlideJoint::getAnchorA() const {
    auto anchor = cpSlideJointGetAnchorA(m_obj);
    return gf::vec(anchor.x, anchor.y);
  }

  void SlideJoint::setAnchorA(gf::Vector2f anchor) {
    cpSlideJointSetAnchorA(m_obj, cpv(anchor.x, anchor.y));
  }

  gf::Vector2f SlideJoint::getAnchorB() const {
    auto anchor = cpSlideJointGetAnchorB(m_obj);
    return gf::vec(anchor.x, anchor.y);
  }

  void SlideJoint::setAnchorB(gf::Vector2f anchor) {
    cpSlideJointSetAnchorB(m_obj, cpv(anchor.x, anchor.y));
  }

  float SlideJoint::getMin() const {
    return cpSlideJointGetMin(m_obj);
  }

  void SlideJoint::setMix(float min) {
    cpSlideJointSetMin(m_obj, min);
  }

  float SlideJoint::getMax() const {
    return cpSlideJointGetMax(m_obj);
  }

  void SlideJoint::setMax(float max) {
    cpSlideJointSetMax(m_obj, max);
  }

  /*
   * PivotJoint
   */

  PivotJoint::PivotJoint(Body a, Body b, gf::Vector2f pivot)
  : Constraint(cpPivotJointNew(unwrap(a), unwrap(b), cpv(pivot.x, pivot.y)))
  {
  }

  PivotJoint::PivotJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB)
  : Constraint(cpPivotJointNew2(unwrap(a), unwrap(b), cpv(anchorA.x, anchorA.y), cpv(anchorB.x, anchorB.y)))
  {
  }

  gf::Vector2f PivotJoint::getAnchorA() const {
    auto anchor = cpPivotJointGetAnchorA(m_obj);
    return gf::vec(anchor.x, anchor.y);
  }

  void PivotJoint::setAnchorA(gf::Vector2f anchor) {
    cpPivotJointSetAnchorA(m_obj, cpv(anchor.x, anchor.y));
  }

  gf::Vector2f PivotJoint::getAnchorB() const {
    auto anchor = cpPivotJointGetAnchorA(m_obj);
    return gf::vec(anchor.x, anchor.y);
  }

  void PivotJoint::setAnchorB(gf::Vector2f anchor) {
    cpPivotJointSetAnchorB(m_obj, cpv(anchor.x, anchor.y));
  }

  /*
   * GrooveJoint
   */

  GrooveJoint::GrooveJoint(Body a, Body b, gf::Vector2f grooveA, gf::Vector2f grooveB, gf::Vector2f anchorB)
  : Constraint(cpGrooveJointNew(unwrap(a), unwrap(b), cpv(grooveA.x, grooveA.y), cpv(grooveB.x, grooveB.y), cpv(anchorB.x, anchorB.y)))
  {
  }

  gf::Vector2f GrooveJoint::getGrooveA() const {
    auto groove = cpGrooveJointGetGrooveA(m_obj);
    return gf::vec(groove.x, groove.y);
  }

  void GrooveJoint::setGrooveA(gf::Vector2f groove) {
    cpGrooveJointSetGrooveA(m_obj, cpv(groove.x, groove.y));
  }

  gf::Vector2f GrooveJoint::getGrooveB() const {
    auto groove = cpGrooveJointGetGrooveB(m_obj);
    return gf::vec(groove.x, groove.y);
  }

  void GrooveJoint::setGrooveB(gf::Vector2f groove) {
    cpGrooveJointSetGrooveB(m_obj, cpv(groove.x, groove.y));
  }

  gf::Vector2f GrooveJoint::getAnchorB() const {
    auto anchor = cpGrooveJointGetAnchorB(m_obj);
    return gf::vec(anchor.x, anchor.y);
  }

  void GrooveJoint::setAnchorB(gf::Vector2f anchor) {
    cpGrooveJointSetAnchorB(m_obj, cpv(anchor.x, anchor.y));
  }

  /*
   * DampedSpring
   */

  DampedSpring::DampedSpring(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB, float restLength, float stiffness, float damping)
  : Constraint(cpDampedSpringNew(unwrap(a), unwrap(b), cpv(anchorA.x, anchorA.y), cpv(anchorB.x, anchorB.y), restLength, stiffness, damping))
  {
  }

  gf::Vector2f DampedSpring::getAnchorA() const {
    auto anchor = cpDampedSpringGetAnchorA(m_obj);
    return gf::vec(anchor.x, anchor.y);
  }

  void DampedSpring::setAnchorA(gf::Vector2f anchor) {
    cpDampedSpringSetAnchorA(m_obj, cpv(anchor.x, anchor.y));
  }

  gf::Vector2f DampedSpring::getAnchorB() const {
    auto anchor = cpDampedSpringGetAnchorB(m_obj);
    return gf::vec(anchor.x, anchor.y);
  }

  void DampedSpring::setAnchorB(gf::Vector2f anchor)  {
    cpDampedSpringSetAnchorB(m_obj, cpv(anchor.x, anchor.y));
  }

  float DampedSpring::getRestLength() const {
    return cpDampedSpringGetRestLength(m_obj);
  }

  void DampedSpring::setRestLength(float restLength) {
    cpDampedSpringSetRestLength(m_obj, restLength);
  }

  float DampedSpring::getStiffness() const {
    return cpDampedSpringGetStiffness(m_obj);
  }

  void DampedSpring::setStiffness(float stiffness) {
    cpDampedSpringSetStiffness(m_obj, stiffness);
  }

  float DampedSpring::getDamping() const {
    return cpDampedSpringGetDamping(m_obj);
  }

  void DampedSpring::setDamping(float damping) {
    cpDampedSpringSetDamping(m_obj, damping);
  }

  /*
   * DampedRotarySpring
   */

  DampedRotarySpring::DampedRotarySpring(Body a, Body b, float restAngle, float stiffness, float damping)
  : Constraint(cpDampedRotarySpringNew(unwrap(a), unwrap(b), restAngle, stiffness, damping))
  {
  }

  float DampedRotarySpring::getRestAngle() const {
    return cpDampedRotarySpringGetRestAngle(m_obj);
  }

  void DampedRotarySpring::setRestAngle(float restAngle) {
    cpDampedRotarySpringSetRestAngle(m_obj, restAngle);
  }

  float DampedRotarySpring::getStiffness() const {
    return cpDampedRotarySpringGetStiffness(m_obj);
  }

  void DampedRotarySpring::setStiffness(float stiffness) {
    cpDampedRotarySpringSetStiffness(m_obj, stiffness);
  }

  float DampedRotarySpring::getDamping() const {
    return cpDampedRotarySpringGetDamping(m_obj);
  }

  void DampedRotarySpring::setDamping(float damping) {
    cpDampedRotarySpringSetDamping(m_obj, damping);
  }

  /*
   * RotaryLimitJoint
   */

  RotaryLimitJoint::RotaryLimitJoint(Body a, Body b, float min, float max)
  : Constraint(cpRotaryLimitJointNew(unwrap(a), unwrap(b), min, max))
  {
  }

  float RotaryLimitJoint::getMin() const {
    return cpRotaryLimitJointGetMin(m_obj);
  }

  void RotaryLimitJoint::setMix(float min) {
    cpRotaryLimitJointSetMin(m_obj, min);
  }

  float RotaryLimitJoint::getMax() const {
    return cpRotaryLimitJointGetMax(m_obj);
  }

  void RotaryLimitJoint::setMax(float max) {
    cpRotaryLimitJointSetMax(m_obj, max);
  }

  /*
   * RatchetJoint
   */

  RatchetJoint::RatchetJoint(Body a, Body b, float phase, float ratchet)
  : Constraint(cpRatchetJointNew(unwrap(a), unwrap(b), phase, ratchet))
  {
  }

  float RatchetJoint::getAngle() const {
    return cpRatchetJointGetAngle(m_obj);
  }
  void RatchetJoint::setAngle(float angle) {
    cpRatchetJointSetAngle(m_obj, angle);
  }

  float RatchetJoint::getPhase() const {
    return cpRatchetJointGetPhase(m_obj);
  }

  void RatchetJoint::setPhase(float phase) {
    cpRatchetJointSetPhase(m_obj, phase);
  }

  float RatchetJoint::getRatchet() const {
    return cpRatchetJointGetRatchet(m_obj);
  }

  void RatchetJoint::setRatchet(float ratchet) {
    cpRatchetJointSetRatchet(m_obj, ratchet);
  }

  /*
   * GearJoint
   */

  GearJoint::GearJoint(Body a, Body b, float phase, float ratio)
  : Constraint(cpGearJointNew(unwrap(a), unwrap(b), phase, ratio))
  {
  }

  float GearJoint::getPhase() const {
    return cpGearJointGetPhase(m_obj);
  }

  void GearJoint::setPhase(float phase) {
    cpGearJointSetPhase(m_obj, phase);
  }

  float GearJoint::getRatio() const {
    return cpGearJointGetRatio(m_obj);
  }

  void GearJoint::setRatio(float ratio) {
    cpGearJointSetRatio(m_obj, ratio);
  }

  /*
   * SimpleMotor
   */

  SimpleMotor::SimpleMotor(Body a, Body b, float rate)
  : Constraint(cpSimpleMotorNew(unwrap(a), unwrap(b), rate))
  {
  }

  float SimpleMotor::getRate() const {
    return cpSimpleMotorGetRate(m_obj);
  }

  void SimpleMotor::setRate(float rate) {
    cpSimpleMotorSetRate(m_obj, rate);
  }

  /*
   * PhysicsFactory
   */

  PhysicsFactory::~PhysicsFactory() {
    for (auto ptr : m_shapes) {
      cpShapeFree(ptr);
    }

    for (auto ptr : m_constraints) {
      cpConstraintFree(ptr);
    }

    for (auto ptr : m_bodies) {
      cpBodyFree(ptr);
    }

    for (auto ptr : m_spaces) {
      cpSpaceFree(ptr);
    }
  }

  Space PhysicsFactory::makeSpace() {
    Space space;
    m_spaces.push_back(space.m_obj);
    return space;
  }

  Body PhysicsFactory::makeBody(float mass, float moment) {
    Body body(mass, moment);
    m_bodies.push_back(body.m_obj);
    return body;
  }

  Body PhysicsFactory::makeKinematicBody() {
    Body body = Body::makeKinematic();
    m_bodies.push_back(body.m_obj);
    return body;
  }

  Body PhysicsFactory::makeStaticBody() {
    Body body = Body::makeStatic();
    m_bodies.push_back(body.m_obj);
    return body;
  }

  PinJoint PhysicsFactory::makePinJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB) {
    PinJoint constraint(a, b, anchorA, anchorB);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  SlideJoint PhysicsFactory::makeSlideJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB, float min, float max) {
    SlideJoint constraint(a, b, anchorA, anchorB, min, max);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  PivotJoint PhysicsFactory::makePivotJoint(Body a, Body b, gf::Vector2f pivot) {
    PivotJoint constraint(a, b, pivot);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  PivotJoint PhysicsFactory::makePivotJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB) {
    PivotJoint constraint(a, b, anchorA, anchorB);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  GrooveJoint PhysicsFactory::makeGrooveJoint(Body a, Body b, gf::Vector2f grooveA, gf::Vector2f grooveB, gf::Vector2f anchorB) {
    GrooveJoint constraint(a, b, grooveA, grooveB, anchorB);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  DampedSpring PhysicsFactory::makeDampedSpring(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB, float restLength, float stiffness, float damping) {
    DampedSpring constraint(a, b, anchorA, anchorB, restLength, stiffness, damping);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  DampedRotarySpring PhysicsFactory::makeDampedRotarySpring(Body a, Body b, float restAngle, float stiffness, float damping) {
    DampedRotarySpring constraint(a, b, restAngle, stiffness, damping);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  RotaryLimitJoint PhysicsFactory::makeRotaryLimitJoint(Body a, Body b, float min, float max) {
    RotaryLimitJoint constraint(a, b, min, max);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  RatchetJoint PhysicsFactory::makeRatchetJoint(Body a, Body b, float phase, float ratchet) {
    RatchetJoint constraint(a, b, phase, ratchet);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  GearJoint PhysicsFactory::makeGearJoint(Body a, Body b, float phase, float ratio) {
    GearJoint constraint(a, b, phase, ratio);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  SimpleMotor PhysicsFactory::makeSimpleMotor(Body a, Body b, float rate) {
    SimpleMotor constraint(a, b, rate);
    m_constraints.push_back(constraint.m_obj);
    return constraint;
  }

  CircleShape PhysicsFactory::makeCircleShape(Body body, float radius, gf::Vector2f offset) {
    CircleShape shape(body, radius, offset);
    m_shapes.push_back(shape.m_obj);
    return shape;
  }

  SegmentShape PhysicsFactory::makeSegmentShape(Body body, gf::Vector2f a, gf::Vector2f b, float radius) {
    SegmentShape shape(body, a, b, radius);
    m_shapes.push_back(shape.m_obj);
    return shape;
  }

  PolygonShape PhysicsFactory::makePolygonShape(Body body, gf::Span<const gf::Vector2f> verts, gf::Matrix3f transform, float radius) {
    PolygonShape shape(body, verts, transform, radius);
    m_shapes.push_back(shape.m_obj);
    return shape;
  }

  PolygonShape PhysicsFactory::makeBoxShape(Body body, gf::RectF box, float radius) {
    PolygonShape shape(body, box, radius);
    m_shapes.push_back(shape.m_obj);
    return shape;
  }

}
