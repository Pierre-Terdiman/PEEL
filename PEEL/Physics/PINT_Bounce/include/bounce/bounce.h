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

#ifndef BOUNCE_H
#define BOUNCE_H

// Include this file header in your project to directly access Bounce objects.

#include <bounce/common/settings.h>
#include <bounce/common/draw.h>

#include <bounce/common/math/math.h>

#include <bounce/collision/gjk/gjk.h>
#include <bounce/collision/sat/sat.h>
#include <bounce/collision/collision.h>
#include <bounce/collision/broad_phase.h>
#include <bounce/collision/time_of_impact.h>

#include <bounce/collision/shapes/sphere.h>
#include <bounce/collision/shapes/capsule.h>
#include <bounce/collision/shapes/hull.h>
#include <bounce/collision/shapes/box_hull.h>
#include <bounce/collision/shapes/cylinder_hull.h>
#include <bounce/collision/shapes/cone_hull.h>
#include <bounce/collision/shapes/mesh.h>
#include <bounce/collision/shapes/grid_mesh.h>
#include <bounce/collision/shapes/sdf.h>

#include <bounce/dynamics/joints/mouse_joint.h>
#include <bounce/dynamics/joints/spring_joint.h>
#include <bounce/dynamics/joints/weld_joint.h>
#include <bounce/dynamics/joints/sphere_joint.h>
#include <bounce/dynamics/joints/revolute_joint.h>
#include <bounce/dynamics/joints/cone_joint.h>
#include <bounce/dynamics/joints/friction_joint.h>
#include <bounce/dynamics/joints/motor_joint.h>
#include <bounce/dynamics/joints/prismatic_joint.h>
#include <bounce/dynamics/joints/wheel_joint.h>

#include <bounce/dynamics/shapes/sphere_shape.h>
#include <bounce/dynamics/shapes/capsule_shape.h>
#include <bounce/dynamics/shapes/triangle_shape.h>
#include <bounce/dynamics/shapes/hull_shape.h>
#include <bounce/dynamics/shapes/mesh_shape.h>

#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/contacts/convex_contact.h>
#include <bounce/dynamics/contacts/mesh_contact.h>

#include <bounce/dynamics/body.h>

#include <bounce/dynamics/world.h>
#include <bounce/dynamics/world_listeners.h>

#include <bounce/rope/rope.h>

#include <bounce/softbody/softbody.h>
#include <bounce/softbody/softbody_particle.h>

#include <bounce/softbody/shapes/softbody_sphere_shape.h>
#include <bounce/softbody/shapes/softbody_capsule_shape.h>
#include <bounce/softbody/shapes/softbody_triangle_shape.h>
#include <bounce/softbody/shapes/softbody_tetrahedron_shape.h>
#include <bounce/softbody/shapes/softbody_world_shape.h>

#include <bounce/softbody/forces/softbody_stretch_force.h>
#include <bounce/softbody/forces/softbody_shear_force.h>
#include <bounce/softbody/forces/softbody_spring_force.h>
#include <bounce/softbody/forces/softbody_mouse_force.h>
#include <bounce/softbody/forces/softbody_triangle_element_force.h>
#include <bounce/softbody/forces/softbody_tetrahedron_element_force.h>

#endif