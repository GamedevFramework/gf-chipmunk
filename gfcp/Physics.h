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
struct cpShape;
struct cpSpace;

namespace gfcp {
  class Body;
//   class Constraint;
  class Shape;

  namespace details {

    template<typename T>
    class PhysicsHandle {
    public:
      PhysicsHandle(T * obj)
      : m_obj(obj)
      {
      }

      PhysicsHandle(const PhysicsHandle&) = delete;

      PhysicsHandle(PhysicsHandle&& other) noexcept
      : m_obj(std::exchange(other.m_obj, nullptr))
      {
      }

      ~PhysicsHandle() = default;

      PhysicsHandle& operator=(const PhysicsHandle&) = delete;

      PhysicsHandle& operator=(PhysicsHandle&& other) {
        std::swap(m_obj, other.m_obj);
        return *this;
      }

      void clear() {
        m_obj = nullptr;
      }

      T * getObject() noexcept {
        return m_obj;
      }

      const T * getObject() const noexcept {
        return m_obj;
      }

      operator T * () noexcept {
        return m_obj;
      }

      operator const T * () const noexcept {
        return m_obj;
      }

    private:
      T * m_obj;
    };

  }

//   class Arbiter {
//   public:
//
//   private:
//     details::PhysicsHandle<cpArbiter> m_handle;
//   };

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
    : m_handle(obj)
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

    // TODO: userdata

    float getCurrentTimeStep() const;

    bool isLocked();

    // TODO: collision handlers

    void addShape(Shape& shape);
    void addBody(Body& body);
    void addBody(Body&& body);
    // void addConstraint(Constraint& constraint);

    void removeShape(Shape& shape);
    void removeBody(Body& body);
    // void removeConstraint(Constraint& constraint);

    bool containsShape(Shape& shape);
    bool containsBody(Body& body);
//     bool containsConstraint(Constraint& constraint);

    // TODO: post-step callback

    // TODO: queries

    void eachBody(std::function<void(Body)> func);
//     void eachShape(std::function<void(Shape)> func);
//     void eachConstraint(std::function<void(Constraint)> func);

    void reindexStatic();
    void reindexShape(Shape& shape);
    void reindexShapesForBody(Body& body);

    void useSpatialHash(float dim, int count);

    void debugDraw(SpaceDebug& debug);

    void update(gf::Time time) override;
  private:

    friend class Body;
    friend class Shape;

  private:
    details::PhysicsHandle<cpSpace> m_handle;
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
    : m_handle(obj)
    {
    }

    Body(float mass, float moment);

    static Body makeKinematic();
    static Body makeStatic();

    void dispose();

    void activate();
    void activateStatic(Shape& filter);

    void sleep();
    void sleepWithGroup(Body& group);
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

    // TODO: userdata

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
//     void eachConstraint(std::function<void(Constraint)> func);
//     void eachArbiter(std::function<void(Arbiter)> func);

  private:
    friend class Space;
    friend class Shape;

    friend class CircleShape;
    friend class SegmentShape;
    friend class PolygonShape;

    details::PhysicsHandle<cpBody> m_handle;
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
    : m_handle(obj)
    {
    }

    void dispose();

    // TODO: update

    // TODO: query

    // TODO: collision (static)

    Space getSpace() const;
    Body getBody() const;
    void setBody(Body& body);

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

    // TODO userdata

    uintptr_t getCollisionType() const;
    void setCollisionType(uintptr_t type);

    ShapeFilter getShapeFilter() const;
    void setShapeFilter(ShapeFilter filter);

  private:
    friend class Space;
    friend class Body;
    friend class PhysicsFactory;

  protected:
    details::PhysicsHandle<cpShape> m_handle;
  };

  class CircleShape : public Shape {
  public:
    CircleShape(Body& body, float radius, gf::Vector2f offset);

    CircleShape(cpShape * obj)
    : Shape(obj)
    {
    }

    gf::Vector2f getOffset() const;
    float getRadius() const;
  };

  class SegmentShape : public Shape {
  public:
    SegmentShape(Body& body, gf::Vector2f a, gf::Vector2f b, float radius);

    SegmentShape(cpShape * obj)
    : Shape(obj)
    {
    }

    void setNeighbors(gf::Vector2f prev, gf::Vector2f next);

    gf::Vector2f getA() const;
    gf::Vector2f getB() const;
    gf::Vector2f getNormal() const;
    float getRadius() const;
  };

  class PolygonShape : public Shape {
  public:
    PolygonShape(Body& body, gf::Span<const gf::Vector2f> verts, gf::Matrix3f transform, float radius);
    PolygonShape(Body& body, gf::RectF box, float radius);

    PolygonShape(cpShape * obj)
    : Shape(obj)
    {
    }

    std::size_t getPointCount() const;
    gf::Vector2f getPoint(std::size_t index) const;
    float getRadius() const;
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

    CircleShape makeCircleShape(Body& body, float radius, gf::Vector2f offset);
    SegmentShape makeSegmentShape(Body& body, gf::Vector2f a, gf::Vector2f b, float radius);
    PolygonShape makePolygonShape(Body& body, gf::Span<const gf::Vector2f> verts, gf::Matrix3f transform, float radius);
    PolygonShape makeBoxShape(Body& body, gf::RectF box, float radius);

  private:
    std::vector<cpSpace*> m_spaces;
    std::vector<cpBody*> m_bodies;
    std::vector<cpShape*> m_shapes;
  };



}

#endif // GFCP_PHYSICS_H
