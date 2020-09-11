#include <gfcp/Physics.h>

#include <chipmunk/chipmunk.h>

#include <gf/Color.h>

namespace gfcp {
  /*
   * Arbiter
   */

  float Arbiter::getRestitution() const {
    return cpArbiterGetRestitution(m_handle);
  }

  void Arbiter::setRestitution(float restitution) {
    cpArbiterSetRestitution(m_handle, restitution);
  }

  float Arbiter::getFriction() const {
    return cpArbiterGetFriction(m_handle);
  }

  void Arbiter::setFriction(float friction) {
    cpArbiterSetFriction(m_handle, friction);
  }

  gf::Vector2f Arbiter::getSurfaceVelocity() {
    auto velocity = cpArbiterGetSurfaceVelocity(m_handle);
    return gf::vec(velocity.x, velocity.y);
  }

  void Arbiter::setSurfaceVelocity(gf::Vector2f velocity) {
    cpArbiterSetSurfaceVelocity(m_handle, cpv(velocity.x, velocity.y));
  }

  gf::Vector2f Arbiter::computeTotalImpulse() const {
    auto impulse = cpArbiterTotalImpulse(m_handle);
    return gf::vec(impulse.x, impulse.y);
  }

  float Arbiter::computeTotalKineticEnergy() const {
    return cpArbiterTotalKE(m_handle);
  }

  bool Arbiter::ignore() {
    return cpArbiterIgnore(m_handle) == cpTrue;
  }

  std::pair<Shape, Shape> Arbiter::getShapes() const {
    cpShape * a;
    cpShape * b;
    cpArbiterGetShapes(m_handle, &a, &b);
    return std::make_pair(Shape(a), Shape(b));
  }

  std::pair<Body, Body> Arbiter::getBodies() const {
    cpBody * a;
    cpBody * b;
    cpArbiterGetBodies(m_handle, &a, &b);
    return std::make_pair(Body(a), Body(b));
  }

  bool Arbiter::isFirstContact() const {
    return cpArbiterIsFirstContact(m_handle) == cpTrue;
  }

  bool Arbiter::isRemoval() const {
    return cpArbiterIsRemoval(m_handle) == cpTrue;
  }

  int Arbiter::getCount() const {
    return cpArbiterGetCount(m_handle);
  }

  gf::Vector2f Arbiter::getNormal() const {
    auto normal = cpArbiterGetNormal(m_handle);
    return gf::vec(normal.x, normal.y);
  }

  gf::Vector2f Arbiter::getPointA(int i) const {
    auto a = cpArbiterGetPointA(m_handle, i);
    return gf::vec(a.x, a.y);
  }

  gf::Vector2f Arbiter::getPointB(int i) const {
    auto b = cpArbiterGetPointB(m_handle, i);
    return gf::vec(b.x, b.y);
  }

  float Arbiter::getDepth(int i) const {
    return cpArbiterGetDepth(m_handle, i);
  }

  bool Arbiter::callWildcardBeginA(Space space) {
    return cpArbiterCallWildcardBeginA(m_handle, space.m_handle);
  }

  bool Arbiter::callWildcardBeginB(Space space) {
    return cpArbiterCallWildcardBeginB(m_handle, space.m_handle);
  }

  bool Arbiter::callWildcardPreSolveA(Space space) {
    return cpArbiterCallWildcardPreSolveA(m_handle, space.m_handle);
  }

  bool Arbiter::callWildcardPreSolveB(Space space) {
    return cpArbiterCallWildcardPreSolveB(m_handle, space.m_handle);
  }

  void Arbiter::callWildcardPostSolveA(Space space) {
    cpArbiterCallWildcardPostSolveA(m_handle, space.m_handle);
  }

  void Arbiter::callWildcardPostSolveB(Space space) {
    cpArbiterCallWildcardPostSolveB(m_handle, space.m_handle);
  }

  void Arbiter::callWildcardSeparateA(Space space) {
    cpArbiterCallWildcardSeparateA(m_handle, space.m_handle);
  }

  void Arbiter::callWildcardSeparateB(Space space) {
    cpArbiterCallWildcardSeparateB(m_handle, space.m_handle);
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
  : m_handle(cpSpaceNew())
  {
  }

  void Space::dispose() {
    cpSpaceFree(m_handle);
    m_handle.clear();
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
    auto space = static_cast<cpSpace*>(m_handle);
    cpSpaceEachShape(space, disposeShapeIterator, static_cast<void*>(space));
    cpSpaceEachConstraint(space, disposeConstraintIterator, static_cast<void*>(space));
    cpSpaceEachBody(space, disposeBodyIterator, static_cast<void*>(space));
  }

  int Space::getIterations() const {
    return cpSpaceGetIterations(m_handle);
  }

  void Space::setIterations(int iterations) {
    cpSpaceSetIterations(m_handle, iterations);
  }

  gf::Vector2f Space::getGravity() const {
    auto gravity = cpSpaceGetGravity(m_handle);
    return gf::vec(gravity.x, gravity.y);
  }

  void Space::setGravity(gf::Vector2f gravity) {
    cpSpaceSetGravity(m_handle, cpv(gravity.x, gravity.y));
  }

  float Space::getDamping() const {
    return cpSpaceGetDamping(m_handle);
  }

  void Space::setDamping(float damping) {
    cpSpaceSetDamping(m_handle, damping);
  }

  float Space::getIdleSpeedThreshold() const {
    return cpSpaceGetIdleSpeedThreshold(m_handle);
  }

  void Space::setIdleSpeedThreshold(float threshold) {
    cpSpaceSetIdleSpeedThreshold(m_handle, threshold);
  }

  float Space::getSleepTimeThreshold() const {
    return cpSpaceGetSleepTimeThreshold(m_handle);
  }

  void Space::setSleepTimeThreshold(float threshold) {
    cpSpaceSetSleepTimeThreshold(m_handle, threshold);
  }

  float Space::getCollisionSlop() const {
    return cpSpaceGetCollisionSlop(m_handle);
  }

  void Space::setCollisionSlop(float slop) {
    cpSpaceSetCollisionSlop(m_handle, slop);
  }

  float Space::getCollisionBias() const {
    return cpSpaceGetCollisionBias(m_handle);
  }

  void Space::setCollisionBias(float bias) {
    cpSpaceSetCollisionBias(m_handle, bias);
  }

  unsigned Space::getCollisionPersistence() const {
    return cpSpaceGetCollisionPersistence(m_handle);
  }

  void Space::setCollisionPersistence(unsigned persistence) {
    cpSpaceSetCollisionPersistence(m_handle, persistence);
  }

  float Space::getCurrentTimeStep() const {
    return cpSpaceGetCurrentTimeStep(m_handle);
  }

  bool Space::isLocked() {
    return cpSpaceIsLocked(m_handle) == cpTrue;
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
    auto raw = cpSpaceAddDefaultCollisionHandler(m_handle);
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

  void Space::addShape(Shape& shape) {
    cpSpaceAddShape(m_handle, shape.m_handle);
  }

  void Space::addBody(Body& body) {
    cpSpaceAddBody(m_handle, body.m_handle);
  }

  void Space::addBody(Body&& body) {
    cpSpaceAddBody(m_handle, body.m_handle);
  }

  void Space::removeShape(Shape& shape) {
    cpSpaceRemoveShape(m_handle, shape.m_handle);
  }

  void Space::removeBody(Body& body) {
    cpSpaceRemoveBody(m_handle, body.m_handle);
  }

  bool Space::containsShape(Shape& shape) {
    return cpSpaceContainsShape(m_handle, shape.m_handle) == cpTrue;
  }

  bool Space::containsBody(Body& body) {
    return cpSpaceContainsBody(m_handle, body.m_handle) == cpTrue;
  }

  namespace {

    void mySpaceBodyIteratorFunc(cpBody * body, void * data) {
      auto func = static_cast<std::function<void(Body)>*>(data);
      (*func)(Body(body));
    }

  }

  void Space::eachBody(std::function<void(Body)> func) {
    cpSpaceEachBody(m_handle, mySpaceBodyIteratorFunc, &func);
  }

  void Space::reindexStatic() {
    cpSpaceReindexStatic(m_handle);
  }

  void Space::reindexShape(Shape& shape) {
    cpSpaceReindexShape(m_handle, shape.m_handle);
  }

  void Space::reindexShapesForBody(Body& body) {
    cpSpaceReindexShapesForBody(m_handle, body.m_handle);
  }

  void Space::useSpatialHash(float dim, int count) {
    cpSpaceUseSpatialHash(m_handle, dim, count);
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
    cpSpaceDebugDraw(m_handle, &options);
  }


  void Space::update(gf::Time time) {
    cpSpaceStep(m_handle, time.asSeconds());
  }

  /*
   * Body
   */

  static_assert(static_cast<int>(BodyType::Dynamic) == static_cast<int>(CP_BODY_TYPE_DYNAMIC), "Body type constant mismatch.");
  static_assert(static_cast<int>(BodyType::Kinematic) == static_cast<int>(CP_BODY_TYPE_KINEMATIC), "Body type constant mismatch.");
  static_assert(static_cast<int>(BodyType::Static) == static_cast<int>(CP_BODY_TYPE_STATIC), "Body type constant mismatch.");

  Body::Body(float mass, float moment)
  : m_handle(cpBodyNew(mass, moment))
  {
  }

  Body Body::makeKinematic() {
    return Body(cpBodyNewKinematic());
  }

  Body Body::makeStatic() {
    return Body(cpBodyNewStatic());
  }

  void Body::dispose() {
    cpBodyFree(m_handle);
    m_handle.clear();
  }

  void Body::activate() {
    cpBodyActivate(m_handle);
  }

  void Body::activateStatic(Shape& filter) {
    cpBodyActivateStatic(m_handle, filter.m_handle);
  }

  void Body::sleep() {
    cpBodySleep(m_handle);
  }

  void Body::sleepWithGroup(Body& group) {
    cpBodySleepWithGroup(m_handle, group.m_handle);
  }

  bool Body::isSleeping() const {
    return cpBodyIsSleeping(m_handle) == cpTrue;
  }


  BodyType Body::getType() {
    return static_cast<BodyType>(cpBodyGetType(m_handle));
  }

  void Body::setType(BodyType type) {
    cpBodySetType(m_handle, static_cast<cpBodyType>(type));
  }

  Space Body::getSpace() const {
    return Space(cpBodyGetSpace(m_handle));
  }

  float Body::getMass() const {
    return cpBodyGetMass(m_handle);
  }

  void Body::setMass(float mass) {
    cpBodySetMass(m_handle, mass);
  }

  float Body::getMoment() const {
    return cpBodyGetMoment(m_handle);
  }

  void Body::setMoment(float moment) {
    cpBodySetMoment(m_handle, moment);
  }

  gf::Vector2f Body::getPosition() const {
    auto position = cpBodyGetPosition(m_handle);
    return gf::vec(position.x, position.y);
  }

  void Body::setPosition(gf::Vector2f position) {
    cpBodySetPosition(m_handle, cpv(position.x, position.y));
  }

  gf::Vector2f Body::getCenterOfGravity() const {
    auto cog = cpBodyGetCenterOfGravity(m_handle);
    return gf::vec(cog.x, cog.y);
  }

  void Body::setCenterOfGravity(gf::Vector2f cog) {
    cpBodySetCenterOfGravity(m_handle, cpv(cog.x, cog.y));
  }

  gf::Vector2f Body::getVelocity() const {
    auto velocity = cpBodyGetVelocity(m_handle);
    return gf::vec(velocity.x, velocity.y);
  }

  void Body::setVelocity(gf::Vector2f velocity) {
    cpBodySetVelocity(m_handle, cpv(velocity.x, velocity.y));
  }

  gf::Vector2f Body::getForce() const {
    auto force = cpBodyGetForce(m_handle);
    return gf::vec(force.x, force.y);
  }

  void Body::setForce(gf::Vector2f force) {
    cpBodySetForce(m_handle, cpv(force.x, force.y));
  }

  float Body::getAngle() const {
    return cpBodyGetAngle(m_handle);
  }

  void Body::setAngle(float angle) {
    cpBodySetAngle(m_handle, angle);
  }

  float Body::getAngularVelocity() const {
    return cpBodyGetAngularVelocity(m_handle);
  }

  void Body::setAngularVelocity(float velocity) {
    cpBodySetAngularVelocity(m_handle, velocity);
  }

  float Body::getTorque() const {
    return cpBodyGetTorque(m_handle);
  }

  void Body::setTorque(float torque) {
    cpBodySetTorque(m_handle, torque);
  }

  gf::Vector2f Body::getRotation() const {
    auto rotation = cpBodyGetRotation(m_handle);
    return gf::vec(rotation.x, rotation.y);
  }

  gf::Vector2f Body::computeLocalToWorldCoordinates(gf::Vector2f point) const {
    auto coords = cpBodyLocalToWorld(m_handle, cpv(point.x, point.y));
    return gf::vec(coords.x, coords.y);
  }

  gf::Vector2f Body::computeWorldToLocalCoordinates(gf::Vector2f point) const {
    auto coords = cpBodyWorldToLocal(m_handle, cpv(point.x, point.y));
    return gf::vec(coords.x, coords.y);
  }

  void Body::applyForceAtWorldPoint(gf::Vector2f force, gf::Vector2f point) {
    cpBodyApplyForceAtWorldPoint(m_handle, cpv(force.x, force.y), cpv(point.x, point.y));
  }

  void Body::applyForceAtLocalPoint(gf::Vector2f force, gf::Vector2f point) {
    cpBodyApplyForceAtLocalPoint(m_handle, cpv(force.x, force.y), cpv(point.x, point.y));
  }

  void Body::applyImpulseAtWorldPoint(gf::Vector2f impulse, gf::Vector2f point) {
    cpBodyApplyImpulseAtWorldPoint(m_handle, cpv(impulse.x, impulse.y), cpv(point.x, point.y));
  }

  void Body::applyImpulseAtLocalPoint(gf::Vector2f impulse, gf::Vector2f point) {
    cpBodyApplyImpulseAtLocalPoint(m_handle, cpv(impulse.x, impulse.y), cpv(point.x, point.y));
  }

  gf::Vector2f Body::getVelocityAtWorldPoint(gf::Vector2f point) const {
    auto velocity = cpBodyGetVelocityAtWorldPoint(m_handle, cpv(point.x, point.y));
    return gf::vec(velocity.x, velocity.y);
  }

  gf::Vector2f Body::getVelocityAtLocalPoint(gf::Vector2f point) const {
    auto velocity = cpBodyGetVelocityAtLocalPoint(m_handle, cpv(point.x, point.y));
    return gf::vec(velocity.x, velocity.y);
  }

  float Body::getKinematicEnergy() const {
    return cpBodyKineticEnergy(m_handle);
  }

  namespace {
    void myBodyShapeIterator(cpBody *body, cpShape *shape, void *data) {
      auto func = static_cast<std::function<void(Body, Shape)>*>(data);
      (*func)(Body(body), Shape(shape));
    }
  }

  void Body::eachShape(std::function<void(Body, Shape)> func) {
    cpBodyEachShape(m_handle, myBodyShapeIterator, &func);
  }

  /*
   * Shape
   */

  void Shape::dispose() {
    cpShapeFree(m_handle);
    m_handle.clear();
  }

  Space Shape::getSpace() const {
    return Space(cpShapeGetSpace(m_handle));
  }

  Body Shape::getBody() const {
    return Body(cpShapeGetBody(m_handle));
  }

  void Shape::setBody(Body& body) {
    cpShapeSetBody(m_handle, body.m_handle);
  }

  float Shape::getMass() {
    return cpShapeGetMass(m_handle);
  }

  void Shape::setMass(float mass) {
    cpShapeSetMass(m_handle, mass);
  }

  float Shape::getDensity() {
    return cpShapeGetDensity(m_handle);
  }

  void Shape::setDensity(float density) {
    cpShapeSetDensity(m_handle, density);
  }

  float Shape::getMoment() {
    return cpShapeGetMoment(m_handle);
  }

  float Shape::getArea() {
    return cpShapeGetArea(m_handle);
  }

  gf::Vector2f Shape::getCenterOfGravity() {
    auto cog = cpShapeGetCenterOfGravity(m_handle);
    return gf::vec(cog.x, cog.y);
  }

  gf::RectF Shape::getBB() const {
    auto bb = cpShapeGetBB(m_handle);
    return gf::RectF::fromMinMax(gf::vec(bb.l, bb.b), gf::vec(bb.r, bb.t));
  }

  bool Shape::getSensor() const {
    return cpShapeGetSensor(m_handle);
  }

  void Shape::setSensor(bool sensor) {
    cpShapeSetSensor(m_handle, sensor);
  }

  float Shape::getElasticity() const {
    return cpShapeGetElasticity(m_handle);
  }

  void Shape::setElasticity(float elasticity) {
    cpShapeSetElasticity(m_handle, elasticity);
  }

  float Shape::getFriction() const {
    return cpShapeGetFriction(m_handle);
  }

  void Shape::setFriction(float friction) {
    cpShapeSetFriction(m_handle, friction);
  }

  gf::Vector2f Shape::getSurfaceVelocity() const {
    auto velocity = cpShapeGetSurfaceVelocity(m_handle);
    return gf::vec(velocity.x, velocity.y);
  }

  void Shape::setSurfaceVelocity(gf::Vector2f velocity) {
    cpShapeSetSurfaceVelocity(m_handle, cpv(velocity.x, velocity.y));
  }

  uintptr_t Shape::getCollisionType() const {
    return cpShapeGetCollisionType(m_handle);
  }

  void Shape::setCollisionType(uintptr_t type) {
    cpShapeSetCollisionType(m_handle, type);
  }

  ShapeFilter Shape::getShapeFilter() const {
    auto filter = cpShapeGetFilter(m_handle);
    return { filter.group, filter.categories, filter.mask };
  }

  void Shape::setShapeFilter(ShapeFilter filter) {
    cpShapeSetFilter(m_handle, cpShapeFilterNew(filter.group, filter.categories, filter.mask));
  }

  /*
   * CircleShape
   */
  CircleShape::CircleShape(Body& body, float radius, gf::Vector2f offset)
  : Shape(cpCircleShapeNew(body.m_handle, radius, cpv(offset.x, offset.y)))
  {
  }

  gf::Vector2f CircleShape::getOffset() const {
    auto offset = cpCircleShapeGetOffset(m_handle);
    return gf::vec(offset.x, offset.y);
  }

  float CircleShape::getRadius() const {
    return cpCircleShapeGetRadius(m_handle);
  }

  SegmentShape::SegmentShape(Body& body, gf::Vector2f a, gf::Vector2f b, float radius)
  : Shape(cpSegmentShapeNew(body.m_handle, cpv(a.x, a.y), cpv(b.x, b.y), radius))
  {
  }

  void SegmentShape::setNeighbors(gf::Vector2f prev, gf::Vector2f next) {
    cpSegmentShapeSetNeighbors(m_handle, cpv(prev.x, prev.y), cpv(next.x, next.y));
  }

  gf::Vector2f SegmentShape::getA() const {
    auto a = cpSegmentShapeGetA(m_handle);
    return gf::vec(a.x, a.y);
  }

  gf::Vector2f SegmentShape::getB() const {
    auto b = cpSegmentShapeGetB(m_handle);
    return gf::vec(b.x, b.y);
  }

  gf::Vector2f SegmentShape::getNormal() const {
    auto n = cpSegmentShapeGetNormal(m_handle);
    return gf::vec(n.x, n.y);
  }

  float SegmentShape::getRadius() const {
    return cpSegmentShapeGetRadius(m_handle);
  }

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

  PolygonShape::PolygonShape(Body& body, gf::Span<const gf::Vector2f> verts, gf::Matrix3f transform, float radius)
  : Shape(createPolygoneShape(body.m_handle, verts, transform, radius))
  {
  }

  PolygonShape::PolygonShape(Body& body, gf::RectF box, float radius)
  : Shape(cpBoxShapeNew2(body.m_handle, cpBBNew(box.min.x, box.min.y, box.max.x, box.max.y), radius))
  {
  }

  std::size_t PolygonShape::getPointCount() const {
    return static_cast<std::size_t>(cpPolyShapeGetCount(m_handle));
  }

  gf::Vector2f PolygonShape::getPoint(std::size_t index) const {
    auto point = cpPolyShapeGetVert(m_handle, static_cast<int>(index));
    return gf::vec(point.x, point.y);
  }

  float PolygonShape::getRadius() const {
    return cpPolyShapeGetRadius(m_handle);
  }

  /*
   * PhysicsFactory
   */

  PhysicsFactory::~PhysicsFactory() {
    for (auto ptr : m_shapes) {
      cpShapeFree(ptr);
    }

    for (auto ptr : m_bodies) {
      cpBodyFree(ptr);
    }

    for (auto ptr : m_spaces) {
      cpSpaceFree(ptr);
    }
  }

  Space PhysicsFactory::makeSpace() {
    auto obj = cpSpaceNew();
    m_spaces.push_back(obj);
    return Space(obj);
  }

  Body PhysicsFactory::makeBody(float mass, float moment) {
    auto obj = cpBodyNew(mass, moment);
    m_bodies.push_back(obj);
    return Body(obj);
  }

  Body PhysicsFactory::makeKinematicBody() {
    auto obj = cpBodyNewKinematic();
    m_bodies.push_back(obj);
    return Body(obj);
  }

  Body PhysicsFactory::makeStaticBody() {
    auto obj = cpBodyNewStatic();
    m_bodies.push_back(obj);
    return Body(obj);
  }

  CircleShape PhysicsFactory::makeCircleShape(Body& body, float radius, gf::Vector2f offset) {
    CircleShape shape(body, radius, offset);
    m_shapes.push_back(shape.m_handle);
    return shape;
  }

  SegmentShape PhysicsFactory::makeSegmentShape(Body& body, gf::Vector2f a, gf::Vector2f b, float radius) {
    SegmentShape shape(body, a, b, radius);
    m_shapes.push_back(shape.m_handle);
    return shape;
  }

  PolygonShape PhysicsFactory::makePolygonShape(Body& body, gf::Span<const gf::Vector2f> verts, gf::Matrix3f transform, float radius) {
    PolygonShape shape(body, verts, transform, radius);
    m_shapes.push_back(shape.m_handle);
    return shape;
  }

  PolygonShape PhysicsFactory::makeBoxShape(Body& body, gf::RectF box, float radius) {
    PolygonShape shape(body, box, radius);
    m_shapes.push_back(shape.m_handle);
    return shape;
  }

}
