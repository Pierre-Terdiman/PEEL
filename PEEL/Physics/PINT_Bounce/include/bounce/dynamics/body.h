/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B3_BODY_H
#define B3_BODY_H

#include <bounce/common/math/vec3.h>
#include <bounce/common/math/mat33.h>
#include <bounce/common/math/quat.h>
#include <bounce/common/math/transform.h>
#include <bounce/common/template/list.h>
#include <bounce/dynamics/time_step.h>

class b3World;
class b3Shape;

struct b3ShapeDef;
struct b3MassData;
struct b3JointEdge;

// Static body: Has zero mass, can be moved manually.
// Kinematic body: Has zero mass, non-zero velocity, can be moved by the solver.
// Dynamic body: Has non-zero mass, non-zero velocity determined by force, can be moved by the solver.
enum b3BodyType 
{
	e_staticBody,
	e_kinematicBody,
	e_dynamicBody
};

// Body definition. 
// This sets some default parameters for creating a rigid body.
// Pass this definition to the world to create a new rigid body.
struct b3BodyDef 
{
	b3BodyDef() 
	{
		type = e_staticBody;
		awake = true;
		allowSleep = true;
		fixedRotationX = false;
		fixedRotationY = false;
		fixedRotationZ = false;
		userData = nullptr;
		position.SetZero();
		orientation.SetIdentity();
		linearVelocity.SetZero();
		angularVelocity.SetZero();
		gravityScale.Set(scalar(1), scalar(1), scalar(1));
		linearDamping.SetZero();
		angularDamping.SetZero();
		linearSleepTolerance = scalar(0.05);
		angularSleepTolerance = scalar(2) / scalar(180) * B3_PI;
	}

	// The body type. 
	// The body types are: static, kinematic, and dynamic.
	b3BodyType type;
	
	// If this flag is enable the body will be created in sleeping state.
	bool awake;
	
	// This flag indicates the body is allowed to sleep under small motion.
	bool allowSleep;

	// If enabled the body is constrained to rotate only around the x-axis.
	bool fixedRotationX;
	
	// If enabled the body is constrained to rotate only around the y-axis.
	bool fixedRotationY;
	
	// If enabled the body is constrained to rotate only around the z-axis.
	bool fixedRotationZ;
	
	// The user data. This pointer usually stores an address to a game entity.
	void* userData;
	
	// The initial position of the body.
	// Units are m (meters).
	b3Vec3 position;
	
	// The initial orientation quaternion of the body.
	// Note: Angle is measured in radians.
	b3Quat orientation;
	
	// The initial linear velocity of the body.
	// Units are m/s (meters per second).
	b3Vec3 linearVelocity;
	
	// The initial angular velocity of the body.
	// Units are rad/s (radians per second).
	b3Vec3 angularVelocity;
	
	// Coefficient of damping for linear motion per axis.
	b3Vec3 linearDamping;

	// Coefficient of damping for angular motion per axis.
	b3Vec3 angularDamping;

	// Gravity scale per axis.
	b3Vec3 gravityScale;

	// Sleep tolerance of linear motion in m/s.
	scalar linearSleepTolerance;

	// Sleep tolerance of angular motion in rad/s.
	scalar angularSleepTolerance;
};

// This class encapsulates a rigid body.
// Rigid bodies are allowed to translate and rotate.
// You can use rigid bodies to create shapes consequently creating a compound body. 
// The inertia of a rigid body is determined by the shapes it contains.
// The shapes are also used for collision detection and response.
// You can connect rigid bodies using joints to remove their relative 
// degrees of freedom.
// You can apply non-stiff forces and torques to a rigid body for controlling its motion or 
// you can manipulate the velocities of the rigid body directly.
// Manipulating the velocities directly is the standard way of making a rigid 
// body follow an animation.
// In a game typically a rigid body is associated with a mesh. The rigid body 
// contains a transform that is used to locate the associated mesh in the world 
// and therefore rendering the mesh at that location.
class b3Body
{
public:
	// Get the type of the body.
	b3BodyType GetType() const;

	// Set the type of the body. 
	// This will reset the current body inertial properties.
	void SetType(b3BodyType type);

	// Get the world the body belongs to.
	const b3World* GetWorld() const;
	b3World* GetWorld();

	// Get the shapes associated with the body.
	const b3List<b3Shape>& GetShapeList() const;
	b3List<b3Shape>& GetShapeList();

	// Create a new shape for the body given the shape definition and return a pointer to its clone.
	// The shape passed to the definition it will be cloned and is not recommended modifying 
	// it inside simulation callbacks. 
	// Therefore you can create shapes on the stack memory.
	b3Shape* CreateShape(const b3ShapeDef& def);
	
	// Get the list of all joints connected to this body.
	const b3List<b3JointEdge>& GetJointList() const;
	b3List<b3JointEdge>& GetJointList();

	// Destroy a given shape from the body.
	void DestroyShape(b3Shape* shape);

	// Get the body sweep.
	const b3Sweep& GetSweep() const;

	// Get the body world transform.
	const b3Transform& GetTransform() const;
	
	// Set the body world transform from a position and orientation quaternion.
	// However, manipulating a body transform during the simulation may cause non-physical behaviour.
	void SetTransform(const b3Vec3& position, const b3Quat& orientation);

	// Set the body world transform from a position and orientation matrix.
	// However, manipulating a body transform during the simulation may cause non-physical behaviour.
	void SetTransform(const b3Vec3& position, const b3Mat33& orientation);

	// Get the position of the world body origin.
	b3Vec3 GetPosition() const;

	// Get the orientation of the world body frame.
	b3Quat GetOrientation() const;

	// Get the world position of the center of mass.
	b3Vec3 GetWorldCenter() const;

	// Get the local position of the center of mass.
	b3Vec3 GetLocalCenter() const;
	
	// Get the velocity at a given point.
	b3Vec3 GetPointVelocity(const b3Vec3& point) const;

	// Get the linear velocity of the center of mass.
	b3Vec3 GetLinearVelocity() const;

	// Set the linear velocity of the center of mass. 
	// If is a non-zero velocity then the body awakes.
	// The body must be dynamic or kinematic.
	void SetLinearVelocity(const b3Vec3& linearVelocity);
	
	// Get the angular velocity of the body frame.
	b3Vec3 GetAngularVelocity() const;

	// Set the angular velocity of the body frame. 
	// If is a non-zero velocity then the body awakes.
	// The body must be dynamic or kinematic.
	void SetAngularVelocity(const b3Vec3& angularVelocity);

	// Apply a force at a specific world point. 
	// If the point isn't the center of mass then a non-zero torque is applied.
	// The body must be dynamic.
	void ApplyForce(const b3Vec3& force, const b3Vec3& point, bool wake);

	// Apply a force at the center of mass. Usually this is a external force. 
	// The body must be dynamic.
	void ApplyForceToCenter(const b3Vec3& force, bool wake);

	// Apply a torque to the body.
	// The body must be dynamic.
	void ApplyTorque(const b3Vec3& torque, bool wake);

	// Apply a linear impulse (linear velocity change) to a specific point (particle) of the body. 
	// If the point isn't the center of mass then a non-zero angular impulse is applied.
	// The body must be dynamic.
	void ApplyLinearImpulse(const b3Vec3& impulse, const b3Vec3& point, bool wake);

	// Apply a angular impulse (angular velocity change) to the body.
	// The body must be dynamic.
	void ApplyAngularImpulse(const b3Vec3& impulse, bool wake);

	// Get the mass of the body. Typically in kg/m^3.
	scalar GetMass() const;

	// Get the inverse mass of the body. Typically in kg/m^3.
	scalar GetInverseMass() const;
	
	// Get the rotational inertia of the body about the local center of mass. Typically in kg/m^3.
	const b3Mat33& GetInertia() const;

	// Get the inverse of the rotational inertia of the body about the world center of mass. Typically in kg/m^3.
	const b3Mat33& GetWorldInverseInertia() const;
	
	// Get this body mass data. 
	// However, the mass data returned by this function contains the mass of the body, 
	// the body local center of mass, and the rotational inertia about the body local center of mass.
	void GetMassData(b3MassData* data) const;

	// Set this body mass data.
	// The supplied mass data must have the new mass of the body, local center of mass of the body,
	// and rotational inertia of the body about the body origin. The last requirement must be fulfilled
	// because the given inertia is shifted to the given local center of mass inside this function.
	void SetMassData(const b3MassData* data);

	// Recalculate this body mass data based on all of its shapes.
	void ResetMass();

	// Get the linear kinetic energy of the body in Joules (kg m^2/s^2).
	scalar GetLinearEnergy() const;

	// Get the angular kinetic energy of the body in Joules (kg m^2/s^2).
	scalar GetAngularEnergy() const;

	// Get the total kinetic energy of the body in Joules (kg m^2/s^2).
	scalar GetEnergy() const;
	
	// Transform a vector to the local space of this body.
	b3Vec3 GetLocalVector(const b3Vec3& vector) const;

	// Transform a vector to the world space.
	b3Vec3 GetWorldVector(const b3Vec3& localVector) const;

	// Transform a point to the local space of this body.
	b3Vec3 GetLocalPoint(const b3Vec3& point) const;

	// Transform a point to the world space.
	b3Vec3 GetWorldPoint(const b3Vec3& localPoint) const;

	// Transform a frame to the local space of this body.
	b3Quat GetLocalFrame(const b3Quat& frame) const;

	// Transform a frame to the world space.
	b3Quat GetWorldFrame(const b3Quat& localFrame) const;

	// Transform a frame to the local space of this body.
	b3Transform GetLocalFrame(const b3Transform& frame) const;

	// Transform a frame to the world space.
	b3Transform GetWorldFrame(const b3Transform& localFrame) const;

	// Get the linear damping of the body. 
	const b3Vec3& GetLinearDamping() const;
	
	// Set the linear damping of the body. Zero is set by default.
	void SetLinearDamping(const b3Vec3& damping);

	// Get the angular damping of the body. 
	const b3Vec3& GetAngularDamping() const;

	// Set the angular damping of the body. Zero is set by default.
	void SetAngularDamping(const b3Vec3& damping);

	// Get the gravity scale of the body. 
	const b3Vec3& GetGravityScale() const;

	// Set the gravity scale of the body. One is set by default.
	void SetGravityScale(const b3Vec3& scale);

	// See if the body is awake.
	bool IsAwake() const;

	// Set the awake status of the body.
	void SetAwake(bool flag);

	// Get the user data associated with the body.
	// The user data is usually a game entity.
	void* GetUserData() const;

	// Set the user data to the body.
	void SetUserData(void* _userData);

	// Set the fixed rotation along the world axes.
	void SetFixedRotation(bool flagX, bool flagY, bool flagZ);

	// Set the linear sleep tolerance in meters per second.
	void SetLinearSleepTolerance(scalar tolerance);

	// Get the linear sleep tolerance in meters per second.
	scalar GetLinearSleepTolerance() const;

	// Set the angular sleep tolerance in radians per second.
	void SetAngularSleepTolerance(scalar tolerance);

	// Get the angular sleep tolerance in radians per second.
	scalar GetAngularSleepTolerance() const;

	// Set if automatic sleeping is allowed.
	void SetSleepingAllowed(bool bit);

	// Is automatic sleeping allowed?
	bool IsSleepingAllowed() const;

	// Get the next body in the world body list.
	const b3Body* GetNext() const;
	b3Body* GetNext();

	// Dump this body to a file.
	void Dump() const;
private:
	friend class b3World;
	friend class b3Island;

	friend class b3Contact;
	friend class b3ConvexContact;
	friend class b3MeshContact;
	friend class b3ContactManager;
	friend class b3ContactSolver;
	
	friend class b3Joint;
	friend class b3JointManager;
	friend class b3JointSolver;
	friend class b3MouseJoint;
	friend class b3SpringJoint;
	friend class b3WeldJoint;
	friend class b3RevoluteJoint;
	friend class b3SphereJoint;
	friend class b3ConeJoint;
	friend class b3FrictionJoint;
	friend class b3MotorJoint;
	friend class b3PrismaticJoint;
	friend class b3WheelJoint;

	friend class b3List<b3Body>;

	friend class b3SoftBody;
	friend class b3SoftBodySolver;
	friend class b3SoftBodyContactSolver;

	friend class b3SoftBody;
	friend class b3SoftBodySolver;
	friend class b3SoftBodyContactSolver;

	enum b3BodyFlags 
	{
		e_awakeFlag = 0x0001,
		e_islandFlag = 0x0002,
		e_autoSleepFlag = 0x0004,
		e_fixedRotationX = 0x0008,
		e_fixedRotationY = 0x0010,
		e_fixedRotationZ = 0x0020
	};

	b3Body(const b3BodyDef& def, b3World* world);
	~b3Body() { }

	// Destroy all shapes associated with the body.
	void DestroyShapes();

	// Destroy all contacts associated with the body.
	void DestroyContacts();

	// Destroy all joints connected to the body.
	void DestroyJoints();

	void SynchronizeTransform();
	void SynchronizeShapes();

	// Check if this body should collide with another.
	bool ShouldCollide(const b3Body* other) const;

	b3BodyType m_type;
	u32 m_islandID;
	u32 m_flags;
	
	// Body sleeping
	scalar m_linearSleepTolerance;
	scalar m_angularSleepTolerance;
	scalar m_sleepTime;

	// The shapes attached to this body.
	b3List<b3Shape> m_shapeList;
	
	// Joint edges for this body joint graph.
	b3List<b3JointEdge> m_jointEdges;

	// User associated data (usually an entity).
	void* m_userData;

	// Body mass.
	scalar m_mass;

	// Inverse body mass.
	scalar m_invMass;
	
	// Inertia about the body local center of mass.
	b3Mat33 m_I;	
	
	// Inverse inertia about the body local center of mass.
	b3Mat33 m_invI;	
	
	// Inverse inertia about the body world center of mass.
	b3Mat33 m_worldInvI;
	
	b3Vec3 m_force;
	b3Vec3 m_torque;
	b3Vec3 m_linearVelocity;
	b3Vec3 m_angularVelocity;
	
	b3Vec3 m_linearDamping;
	b3Vec3 m_angularDamping;
	b3Vec3 m_gravityScale;
	
	// Motion proxy for CCD.
	b3Sweep m_sweep;

	// The body origin transform. 
	b3Transform m_xf;
		
	// The parent world of this body.
	b3World* m_world;
	
	// Links to the world body list.
	b3Body* m_prev;
	b3Body* m_next;
};

inline const b3Body* b3Body::GetNext() const
{
	return m_next;
}

inline b3Body* b3Body::GetNext()
{
	return m_next;
}

inline const b3World* b3Body::GetWorld() const 
{
	return m_world;
}

inline b3World* b3Body::GetWorld() 
{
	return m_world;
}

inline b3BodyType b3Body::GetType() const 
{ 
	return m_type; 
}

inline void* b3Body::GetUserData() const 
{ 
	return m_userData; 
}

inline void b3Body::SetUserData(void* userData) 
{ 
	m_userData = userData; 
}

inline const b3List<b3Shape>& b3Body::GetShapeList() const
{
	return m_shapeList;
}

inline b3List<b3Shape>& b3Body::GetShapeList()
{
	return m_shapeList;
}

inline const b3List<b3JointEdge>& b3Body::GetJointList() const
{
	return m_jointEdges;
}

inline b3List<b3JointEdge>& b3Body::GetJointList()
{
	return m_jointEdges;
}

inline const b3Transform& b3Body::GetTransform() const
{
	return m_xf;
}

inline void b3Body::SetTransform(const b3Vec3& position, const b3Quat& orientation)
{
	m_xf.translation = position;
	m_xf.rotation = orientation;

	m_sweep.worldCenter = b3Mul(m_xf, m_sweep.localCenter);
	m_sweep.orientation = orientation;

	m_sweep.worldCenter0 = m_sweep.worldCenter;
	m_sweep.orientation0 = m_sweep.orientation;

	m_worldInvI = b3RotateToFrame(m_invI, orientation);

	SynchronizeShapes();
}

inline void b3Body::SetTransform(const b3Vec3& position, const b3Mat33& orientation)
{
	m_xf.translation = position;
	m_xf.rotation = b3Mat33Quat(orientation);

	m_sweep.worldCenter = b3Mul(m_xf, m_sweep.localCenter);
	m_sweep.orientation = m_xf.rotation;

	m_sweep.worldCenter0 = m_sweep.worldCenter;
	m_sweep.orientation0 = m_sweep.orientation;

	m_worldInvI = b3RotateToFrame(m_invI, orientation);

	SynchronizeShapes();
}

inline b3Vec3 b3Body::GetPosition() const
{
	return m_xf.translation;
}

inline b3Quat b3Body::GetOrientation() const
{
	return m_sweep.orientation;
}

inline b3Vec3 b3Body::GetWorldCenter() const
{
	return m_sweep.worldCenter;
}

inline b3Vec3 b3Body::GetLocalCenter() const
{
	return m_sweep.localCenter;
}

inline b3Vec3 b3Body::GetLocalVector(const b3Vec3& vector) const
{
	return b3MulC(m_xf.rotation, vector);
}

inline b3Vec3 b3Body::GetWorldVector(const b3Vec3& localVector) const
{
	return b3Mul(m_xf.rotation, localVector);
}

inline b3Vec3 b3Body::GetLocalPoint(const b3Vec3& point) const
{
	return b3MulT(m_xf, point);
}

inline b3Vec3 b3Body::GetWorldPoint(const b3Vec3& point) const
{
	return b3Mul(m_xf, point);
}

inline b3Quat b3Body::GetLocalFrame(const b3Quat& frame) const
{
	return b3MulC(m_sweep.orientation, frame);
}

inline b3Quat b3Body::GetWorldFrame(const b3Quat& localFrame) const
{
	return b3Mul(m_sweep.orientation, localFrame);
}

inline b3Transform b3Body::GetLocalFrame(const b3Transform& xf) const
{
	return b3MulT(m_xf, xf);
}

inline b3Transform b3Body::GetWorldFrame(const b3Transform& xf) const
{
	return b3Mul(m_xf, xf);
}

inline const b3Sweep& b3Body::GetSweep() const
{
	return m_sweep;
}

inline bool b3Body::IsAwake() const
{
	return (m_flags & e_awakeFlag) != 0;
}

inline void b3Body::SetAwake(bool flag) 
{
	if (flag) 
	{
		if (!IsAwake()) 
		{
			m_flags |= e_awakeFlag;
			m_sleepTime = scalar(0);
		}
	}
	else 
	{
		m_flags &= ~e_awakeFlag;
		m_sleepTime = scalar(0);
		m_force.SetZero();
		m_torque.SetZero();
		m_linearVelocity.SetZero();
		m_angularVelocity.SetZero();		
	}
}

inline const b3Vec3& b3Body::GetLinearDamping() const
{
	return m_linearDamping;
}

inline void b3Body::SetLinearDamping(const b3Vec3& damping) 
{
	m_linearDamping = damping;
}

inline const b3Vec3& b3Body::GetAngularDamping() const
{
	return m_angularDamping;
}

inline void b3Body::SetAngularDamping(const b3Vec3& damping) 
{
	m_angularDamping = damping;
}

inline const b3Vec3& b3Body::GetGravityScale() const
{ 
	return m_gravityScale; 
}

inline void b3Body::SetGravityScale(const b3Vec3& scale)
{
	if (m_type != e_staticBody) 
	{
		m_gravityScale = scale;
	}
}

inline b3Vec3 b3Body::GetPointVelocity(const b3Vec3& point) const
{
	return m_linearVelocity + b3Cross(m_angularVelocity, point - m_sweep.worldCenter);
}

inline b3Vec3 b3Body::GetLinearVelocity() const
{
	return m_linearVelocity;
}

inline void b3Body::SetLinearVelocity(const b3Vec3& linearVelocity)
{
	if (m_type == e_staticBody) 
	{
		return;
	}
	
	if (b3Dot(linearVelocity, linearVelocity) > scalar(0)) 
	{
		SetAwake(true);
	}

	m_linearVelocity = linearVelocity;
}

inline b3Vec3 b3Body::GetAngularVelocity() const
{
	return m_angularVelocity;
}

inline void b3Body::SetAngularVelocity(const b3Vec3& angularVelocity) 
{
	if (m_type == e_staticBody) 
	{
		return;
	}

	if (b3Dot(angularVelocity, angularVelocity) > scalar(0)) 
	{
		SetAwake(true);
	}

	m_angularVelocity = angularVelocity;
}

inline scalar b3Body::GetMass() const
{
	return m_mass;
}

inline scalar b3Body::GetInverseMass() const
{
	return m_invMass;
}

inline const b3Mat33& b3Body::GetWorldInverseInertia() const
{
	return m_worldInvI;
}

inline const b3Mat33& b3Body::GetInertia() const
{
	return m_I;
}

inline scalar b3Body::GetLinearEnergy() const
{
	b3Vec3 P = m_mass * m_linearVelocity;
	return b3Dot(P, m_linearVelocity);
}

inline scalar b3Body::GetAngularEnergy() const
{
	b3Mat33 I = b3RotateToFrame(m_I, m_xf.rotation);
	b3Vec3 L = I * m_angularVelocity;
	return b3Dot(L, m_angularVelocity);
}

inline scalar b3Body::GetEnergy() const
{
	scalar e1 = GetLinearEnergy();
	scalar e2 = GetAngularEnergy();
	return scalar(0.5) * (e1 + e2);
}

inline void b3Body::ApplyForce(const b3Vec3& force, const b3Vec3& point, bool wake) 
{
	if (m_type != e_dynamicBody) 
	{
		return;
	}

	if (wake && !IsAwake()) 
	{
		SetAwake(true);
	}

	if (IsAwake()) 
	{
		m_force += force;
		m_torque += b3Cross(point - m_sweep.worldCenter, force);
	}
}

inline void b3Body::ApplyForceToCenter(const b3Vec3& force, bool wake) 
{
	if (m_type != e_dynamicBody) 
	{
		return;
	}

	if (wake && !IsAwake()) 
	{
		SetAwake(true);
	}

	if (IsAwake()) 
	{
		m_force += force;
	}
}

inline void b3Body::ApplyTorque(const b3Vec3& torque, bool wake) 
{
	if (m_type != e_dynamicBody) 
	{
		return;
	}

	if (wake && !IsAwake()) 
	{
		SetAwake(true);
	}

	if (IsAwake()) 
	{
		m_torque += torque;
	}
}

inline void b3Body::ApplyLinearImpulse(const b3Vec3& impulse, const b3Vec3& worldPoint, bool wake) 
{
	if (m_type != e_dynamicBody) 
	{
		return;
	}

	if (wake && !IsAwake()) 
	{
		SetAwake(true);
	}

	if (IsAwake()) 
	{
		m_linearVelocity += m_invMass * impulse;
		m_angularVelocity += b3Mul(m_worldInvI, b3Cross(worldPoint - m_sweep.worldCenter, impulse));
	}
}

inline void b3Body::ApplyAngularImpulse(const b3Vec3& impulse, bool wake) 
{
	if (m_type != e_dynamicBody) 
	{
		return;
	}

	if (wake && !IsAwake()) 
	{
		SetAwake(true);
	}

	if (IsAwake()) 
	{
		m_angularVelocity += b3Mul(m_worldInvI, impulse);
	}
}

inline scalar b3Body::GetLinearSleepTolerance() const
{
	return m_linearSleepTolerance;
}

inline scalar b3Body::GetAngularSleepTolerance() const
{
	return m_angularSleepTolerance;
}

inline void b3Body::SetSleepingAllowed(bool flag)
{
	if (flag)
	{
		m_flags |= e_autoSleepFlag;
	}
	else
	{
		m_flags &= ~e_autoSleepFlag;
		SetAwake(true);
	}
}

inline bool b3Body::IsSleepingAllowed() const
{
	return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
}

#endif
