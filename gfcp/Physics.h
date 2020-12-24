#ifndef GFCP_PHYSICS_H
#define GFCP_PHYSICS_H

#include <functional>
#include <utility>

#include <gf/Matrix.h>
#include <gf/Model.h>
#include <gf/Rect.h>
#include <gf/Span.h>
#include <gf/Vector.h>

struct cpArbiter;
struct cpBody;
struct cpConstraint;
struct cpShape;
struct cpSpace;

namespace gfcp {
  class Body;
  class Constraint;
  class Shape;
  class Space;

  float computeMomentForCircle(float m, float r1, float r2, gf::Vector2f offset);
  float computeMomentForSegment(float m, gf::Vector2f a, gf::Vector2f b, float radius);
  float computeMomentForPoly(float m, gf::Span<const gf::Vector2f> verts, gf::Vector2f offset, float radius);
  float computeMomentForBox(float m, gf::RectF box);

  class Arbiter {
  public:
    Arbiter(cpArbiter * obj)
    : m_obj(obj)
    {
    }

    float getRestitution() const;
    void setRestitution(float restitution);

    float getFriction() const;
    void setFriction(float friction);

    gf::Vector2f getSurfaceVelocity();
    void setSurfaceVelocity(gf::Vector2f velocity);

    template<typename T>
    T & getUserData() {
      return *static_cast<T*>(getRawUserData());
    }

    template<typename T>
    void setUserData(T & data) {
      setUserData(std::addressof(data));
    }

    gf::Vector2f computeTotalImpulse() const;
    float computeTotalKineticEnergy() const;

    bool ignore();

    std::pair<Shape, Shape> getShapes() const;
    std::pair<Body, Body> getBodies() const;

    // TODO: contact point set

    bool isFirstContact() const;
    bool isRemoval() const;

    int getCount() const;
    gf::Vector2f getNormal() const;
    gf::Vector2f getPointA(int i) const;
    gf::Vector2f getPointB(int i) const;
    float getDepth(int i) const;

    bool callWildcardBeginA(Space space);
    bool callWildcardBeginB(Space space);

    bool callWildcardPreSolveA(Space space);
    bool callWildcardPreSolveB(Space space);

    void callWildcardPostSolveA(Space space);
    void callWildcardPostSolveB(Space space);

    void callWildcardSeparateA(Space space);
    void callWildcardSeparateB(Space space);

  private:
    void * getRawUserData() const;
    void setRawUserData(void * data);

  private:
    cpArbiter * m_obj;
  };

  class CollisionHandler {
  public:
    virtual ~CollisionHandler() = default;
    virtual bool begin(Arbiter arbiter, Space space);
    virtual bool preSolve(Arbiter arbiter, Space space);
    virtual void postSolve(Arbiter arbiter, Space space);
    virtual void separate(Arbiter arbiter, Space space);
  };

  class SpaceDebug {
  public:
    virtual ~SpaceDebug() = default;
    virtual void drawCircle(gf::Vector2f pos, float angle, float radius, gf::Color4f outlineColor, gf::Color4f fillColor);
    virtual void drawSegment(gf::Vector2f a, gf::Vector2f b, gf::Color4f color);
    virtual void drawFatSegment(gf::Vector2f a, gf::Vector2f b, float radius, gf::Color4f outlineColor, gf::Color4f fillColor);
    virtual void drawPolygon(gf::Span<const gf::Vector2f> verts, float radius, gf::Color4f outlineColor, gf::Color4f fillColor);
    virtual void drawDot(float size, gf::Vector2f pos, gf::Color4f color);
    virtual gf::Color4f getColorForShape(Shape shape);
  };

  class Space : public gf::Model {
  public:
    Space();

    explicit
    Space(cpSpace *obj)
    : m_obj(obj)
    {
    }

    void dispose();
    void disposeChildren();

    int getIterations() const;
    void setIterations(int iterations);

    gf::Vector2f getGravity() const;
    void setGravity(gf::Vector2f gravity);

    float getDamping() const;
    void setDamping(float damping);

    float getIdleSpeedThreshold() const;
    void setIdleSpeedThreshold(float threshold);

    float getSleepTimeThreshold() const;
    void setSleepTimeThreshold(float threshold);

    float getCollisionSlop() const;
    void setCollisionSlop(float slop);

    float getCollisionBias() const;
    void setCollisionBias(float bias);

    unsigned getCollisionPersistence() const;
    void setCollisionPersistence(unsigned persistence);

    template<typename T>
    T & getUserData() {
      return *static_cast<T*>(getRawUserData());
    }

    template<typename T>
    void setUserData(T & data) {
      setUserData(std::addressof(data));
    }

    float getCurrentTimeStep() const;

    bool isLocked();

    void setDefaultCollisionHandler(CollisionHandler& handler);
    void setCollisionHandler(CollisionHandler& handler, uintptr_t a, uintptr_t b);
    void setWildcardHandler(CollisionHandler& handler, uintptr_t type);

    void addShape(Shape shape);
    void addBody(Body body);
    void addConstraint(Constraint constraint);

    void removeShape(Shape shape);
    void removeBody(Body body);
    void removeConstraint(Constraint constraint);

    bool containsShape(Shape shape);
    bool containsBody(Body body);
    bool containsConstraint(Constraint constraint);

    // TODO: post-step callback

    // TODO: queries

    void eachBody(std::function<void(Body)> func);
    void eachShape(std::function<void(Shape)> func);
    void eachConstraint(std::function<void(Constraint)> func);

    void reindexStatic();
    void reindexShape(Shape& shape);
    void reindexShapesForBody(Body& body);

    void useSpatialHash(float dim, int count);

    void debugDraw(SpaceDebug& debug);

    void update(gf::Time time) override;
  private:
    friend class Arbiter;
    friend class Body;
    friend class PhysicsFactory;
    friend class Shape;

  private:
    void * getRawUserData() const;
    void setRawUserData(void * data);

  private:
    cpSpace * m_obj;
  };


  enum class BodyType {
    Dynamic,
    Kinematic,
    Static,
  };

  class Body {
  public:
    explicit
    Body(cpBody * obj)
    : m_obj(obj)
    {
    }

    Body(float mass, float moment);

    static Body makeKinematic();
    static Body makeStatic();

    void dispose();

    void activate();
    void activateStatic(Shape filter);

    void sleep();
    void sleepWithGroup(Body group);
    bool isSleeping() const;

    BodyType getType();
    void setType(BodyType type);

    Space getSpace() const;

    float getMass() const;
    void setMass(float mass);

    float getMoment() const;
    void setMoment(float moment);

    gf::Vector2f getPosition() const;
    void setPosition(gf::Vector2f position);

    gf::Vector2f getCenterOfGravity() const;
    void setCenterOfGravity(gf::Vector2f cog);

    gf::Vector2f getVelocity() const;
    void setVelocity(gf::Vector2f velocity);

    gf::Vector2f getForce() const;
    void setForce(gf::Vector2f force);

    float getAngle() const;
    void setAngle(float angle);

    float getAngularVelocity() const;
    void setAngularVelocity(float velocity);

    float getTorque() const;
    void setTorque(float torque);

    gf::Vector2f getRotation() const;

    template<typename T>
    T & getUserData() {
      return *static_cast<T*>(getRawUserData());
    }

    template<typename T>
    void setUserData(T & data) {
      setUserData(std::addressof(data));
    }

    // TODO: update func (?)

    // TODO: integration function

    gf::Vector2f computeLocalToWorldCoordinates(gf::Vector2f point) const;
    gf::Vector2f computeWorldToLocalCoordinates(gf::Vector2f point) const;

    void applyForceAtWorldPoint(gf::Vector2f force, gf::Vector2f point);
    void applyForceAtLocalPoint(gf::Vector2f force, gf::Vector2f point);

    void applyImpulseAtWorldPoint(gf::Vector2f impulse, gf::Vector2f point);
    void applyImpulseAtLocalPoint(gf::Vector2f impulse, gf::Vector2f point);

    gf::Vector2f getVelocityAtWorldPoint(gf::Vector2f point) const;
    gf::Vector2f getVelocityAtLocalPoint(gf::Vector2f point) const;

    float getKinematicEnergy() const;

    void eachShape(std::function<void(Body, Shape)> func);
    void eachConstraint(std::function<void(Body, Constraint)> func);
    void eachArbiter(std::function<void(Body, Arbiter)> func);

  private:
    friend class Constraint;
    friend class PhysicsFactory;
    friend class Space;
    friend class Shape;

  private:
    void * getRawUserData() const;
    void setRawUserData(void * data);

  private:
    cpBody * m_obj;
  };

  struct ShapeFilter {
    uintptr_t group = 0;
    unsigned categories = 0;
    unsigned mask = 0;
  };

  constexpr ShapeFilter ShapeFilterAll = { 0, ~0u, ~0u };
  constexpr ShapeFilter ShapeFilterNone = { 0, 0u, 0u };

  class Shape {
  public:
    explicit
    Shape(cpShape * obj)
    : m_obj(obj)
    {
    }

    void dispose();

    // TODO: update

    // TODO: query

    // TODO: collision (static)

    Space getSpace() const;
    Body getBody() const;
    void setBody(Body body);

    float getMass();
    void setMass(float mass);

    float getDensity();
    void setDensity(float density);

    float getMoment();
    float getArea();
    gf::Vector2f getCenterOfGravity();

    gf::RectF getBB() const;

    bool getSensor() const;
    void setSensor(bool sensor);

    float getElasticity() const;
    void setElasticity(float elasticity);

    float getFriction() const;
    void setFriction(float friction);

    gf::Vector2f getSurfaceVelocity() const;
    void setSurfaceVelocity(gf::Vector2f velocity);

    template<typename T>
    T & getUserData() {
      return *static_cast<T*>(getRawUserData());
    }

    template<typename T>
    void setUserData(T & data) {
      setUserData(std::addressof(data));
    }

    uintptr_t getCollisionType() const;
    void setCollisionType(uintptr_t type);

    ShapeFilter getShapeFilter() const;
    void setShapeFilter(ShapeFilter filter);

  private:
    friend class Space;
    friend class Body;
    friend class PhysicsFactory;

  protected:
    cpBody * unwrap(Body body);

    void * getRawUserData() const;
    void setRawUserData(void * data);

  protected:
    cpShape * m_obj;
  };

  class CircleShape : public Shape {
  public:
    CircleShape(Body body, float radius, gf::Vector2f offset);

    gf::Vector2f getOffset() const;
    float getRadius() const;
  };

  class SegmentShape : public Shape {
  public:
    SegmentShape(Body body, gf::Vector2f a, gf::Vector2f b, float radius);

    void setNeighbors(gf::Vector2f prev, gf::Vector2f next);

    gf::Vector2f getA() const;
    gf::Vector2f getB() const;
    gf::Vector2f getNormal() const;
    float getRadius() const;
  };

  class PolygonShape : public Shape {
  public:
    PolygonShape(Body body, gf::Span<const gf::Vector2f> verts, gf::Matrix3f transform, float radius);
    PolygonShape(Body body, gf::RectF box, float radius);

    std::size_t getPointCount() const;
    gf::Vector2f getPoint(std::size_t index) const;
    float getRadius() const;
  };


  class Constraint {
  public:
    Constraint(cpConstraint * obj)
    : m_obj(obj)
    {
    }

    void dispose();

    Space getSpace() const;
    Body getBodyA() const;
    Body getBodyB() const;

    float getMaxForce() const;
    void setMaxForce(float force);

    float getErrorBias() const;
    void setErrorBias(float bias);

    float getMaxBias() const;
    void setMaxBias(float bias);

    bool getCollideBodies() const;
    void setCollideBodies(bool collide);

    // TODO: pre-solve and post-solve

    template<typename T>
    T & getUserData() {
      return *static_cast<T*>(getRawUserData());
    }

    template<typename T>
    void setUserData(T & data) {
      setUserData(std::addressof(data));
    }

    float getImpulse();

  private:
    friend class Space;
    friend class PhysicsFactory;

  protected:
    cpBody * unwrap(Body body);

    void * getRawUserData() const;
    void setRawUserData(void * data);

  protected:
    cpConstraint * m_obj;
  };

  class PinJoint : public Constraint {
  public:
    PinJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB);

    gf::Vector2f getAnchorA() const;
    void setAnchorA(gf::Vector2f anchor);

    gf::Vector2f getAnchorB() const;
    void setAnchorB(gf::Vector2f anchor);

    float getDistance() const;
    void setDistance(float distance);
  };

  class SlideJoint : public Constraint {
  public:
    SlideJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB, float min, float max);

    gf::Vector2f getAnchorA() const;
    void setAnchorA(gf::Vector2f anchor);

    gf::Vector2f getAnchorB() const;
    void setAnchorB(gf::Vector2f anchor);

    float getMin() const;
    void setMix(float min);

    float getMax() const;
    void setMax(float max);
  };

  class PivotJoint : public Constraint {
  public:
    PivotJoint(Body a, Body b, gf::Vector2f pivot);
    PivotJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB);

    gf::Vector2f getAnchorA() const;
    void setAnchorA(gf::Vector2f anchor);

    gf::Vector2f getAnchorB() const;
    void setAnchorB(gf::Vector2f anchor);
  };

  class GrooveJoint : public Constraint {
  public:
    GrooveJoint(Body a, Body b, gf::Vector2f grooveA, gf::Vector2f grooveB, gf::Vector2f anchorB);

    gf::Vector2f getGrooveA() const;
    void setGrooveA(gf::Vector2f groove);

    gf::Vector2f getGrooveB() const;
    void setGrooveB(gf::Vector2f groove);

    gf::Vector2f getAnchorB() const;
    void setAnchorB(gf::Vector2f anchor);
  };

  class DampedSpring : public Constraint {
  public:
    DampedSpring(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB, float restLength, float stiffness, float damping);

    gf::Vector2f getAnchorA() const;
    void setAnchorA(gf::Vector2f anchor);

    gf::Vector2f getAnchorB() const;
    void setAnchorB(gf::Vector2f anchor);

    float getRestLength() const;
    void setRestLength(float restLength);

    float getStiffness() const;
    void setStiffness(float stiffness);

    float getDamping() const;
    void setDamping(float damping);

    // TODO: spring force func
  };

  class DampedRotarySpring : public Constraint {
  public:
    DampedRotarySpring(Body a, Body b, float restAngle, float stiffness, float damping);

    float getRestAngle() const;
    void setRestAngle(float restAngle);

    float getStiffness() const;
    void setStiffness(float stiffness);

    float getDamping() const;
    void setDamping(float damping);

    // TODO: spring torque func
  };

  class RotaryLimitJoint : public Constraint {
  public:
    RotaryLimitJoint(Body a, Body b, float min, float max);

    float getMin() const;
    void setMix(float min);

    float getMax() const;
    void setMax(float max);
  };

  class RatchetJoint : public Constraint {
  public:
    RatchetJoint(Body a, Body b, float phase, float ratchet);

    float getAngle() const;
    void setAngle(float angle);

    float getPhase() const;
    void setPhase(float phase);

    float getRatchet() const;
    void setRatchet(float ratchet);
  };

  class GearJoint : public Constraint {
  public:
    GearJoint(Body a, Body b, float phase, float ratio);

    float getPhase() const;
    void setPhase(float phase);

    float getRatio() const;
    void setRatio(float ratio);
  };

  class SimpleMotor : public Constraint {
  public:
    SimpleMotor(Body a, Body b, float rate);

    float getRate() const;
    void setRate(float rate);
  };

  class PhysicsFactory {
  public:
    ~PhysicsFactory();

    PhysicsFactory(const PhysicsFactory&) = delete;
    PhysicsFactory(PhysicsFactory&&) = default;

    PhysicsFactory& operator=(const PhysicsFactory&) = delete;
    PhysicsFactory& operator=(PhysicsFactory&&) = default;

    Space makeSpace();

    Body makeBody(float mass, float moment);
    Body makeKinematicBody();
    Body makeStaticBody();

    PinJoint makePinJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB);
    SlideJoint makeSlideJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB, float min, float max);
    PivotJoint makePivotJoint(Body a, Body b, gf::Vector2f pivot);
    PivotJoint makePivotJoint(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB);
    GrooveJoint makeGrooveJoint(Body a, Body b, gf::Vector2f grooveA, gf::Vector2f grooveB, gf::Vector2f anchorB);
    DampedSpring makeDampedSpring(Body a, Body b, gf::Vector2f anchorA, gf::Vector2f anchorB, float restLength, float stiffness, float damping);
    DampedRotarySpring makeDampedRotarySpring(Body a, Body b, float restAngle, float stiffness, float damping);
    RotaryLimitJoint makeRotaryLimitJoint(Body a, Body b, float min, float max);
    RatchetJoint makeRatchetJoint(Body a, Body b, float phase, float ratchet);
    GearJoint makeGearJoint(Body a, Body b, float phase, float ratio);
    SimpleMotor makeSimpleMotor(Body a, Body b, float rate);

    CircleShape makeCircleShape(Body body, float radius, gf::Vector2f offset);
    SegmentShape makeSegmentShape(Body body, gf::Vector2f a, gf::Vector2f b, float radius);
    PolygonShape makePolygonShape(Body body, gf::Span<const gf::Vector2f> verts, gf::Matrix3f transform, float radius);
    PolygonShape makeBoxShape(Body body, gf::RectF box, float radius);

  private:
    std::vector<cpSpace*> m_spaces;
    std::vector<cpBody*> m_bodies;
    std::vector<cpConstraint*> m_constraints;
    std::vector<cpShape*> m_shapes;
  };

}

#endif // GFCP_PHYSICS_H
