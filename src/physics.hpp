// physics.hpp - rigid body physics for pencil
// SPDX-License-Identifier: Apache-2.0
//
// full rigid body simulation: linear + angular dynamics,
// body-body sphere collisions, ground collision with restitution.

#pragma once

#include "vec3.hpp"

#include <cmath>
#include <vector>

namespace pencil {

struct PhysicsBody {
    Vec3 position;
    Vec3 velocity = {0, 0, 0};
    Vec3 angular_velocity = {0, 0, 0}; // rad/s
    Vec3 rotation = {0, 0, 0};          // accumulated euler angles
    float mass = 1.0f;
    float radius = 0.5f;
    float bounce = 0.5f;
    float friction = 0.2f;
    bool active = true;
    bool is_static = false;
};

class PhysicsWorld {
public:
    float gravity = 9.81f;
    float ground_y = -1.0f;
    std::vector<PhysicsBody> bodies;

    void add_body(const PhysicsBody& b) { bodies.push_back(b); }

    void step(float dt) {
        int substeps = 4;
        float sub_dt = dt / substeps;
        for (int s = 0; s < substeps; ++s) {
            integrate(sub_dt);
            collide_ground();
            collide_bodies();
        }
        // update rotation from angular velocity
        for (auto& b : bodies) {
            if (!b.active || b.is_static) continue;
            b.rotation += b.angular_velocity * dt;
            // damping
            b.angular_velocity *= (1.0f - 0.5f * dt);
        }
    }

    size_t body_count() const { return bodies.size(); }

private:
    void integrate(float dt) {
        for (auto& b : bodies) {
            if (!b.active || b.is_static) continue;
            // linear: apply gravity
            b.velocity.y -= gravity * dt;
            b.position += b.velocity * dt;
            // angular: spin down from friction
            b.angular_velocity *= (1.0f - 0.1f * dt);
        }
    }

    void collide_ground() {
        for (auto& b : bodies) {
            if (!b.active || b.is_static) continue;
            float floor = ground_y + b.radius;
            if (b.position.y < floor) {
                b.position.y = floor;
                if (b.velocity.y < 0) {
                    b.velocity.y = -b.velocity.y * b.bounce;
                    // friction on horizontal velocity
                    b.velocity.x *= (1.0f - b.friction);
                    b.velocity.z *= (1.0f - b.friction);
                    // add spin from sliding
                    b.angular_velocity.x += b.velocity.z * 2.0f;
                    b.angular_velocity.z -= b.velocity.x * 2.0f;
                    // stop if very slow
                    if (std::fabs(b.velocity.y) < 0.3f && 
                        b.velocity.length_squared() < 0.5f) {
                        b.velocity = {0, 0, 0};
                        b.angular_velocity *= 0.5f;
                        if (b.velocity.length_squared() < 0.01f) {
                            b.active = false;
                        }
                    }
                }
            }
        }
    }

    void collide_bodies() {
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                PhysicsBody& a = bodies[i];
                PhysicsBody& b = bodies[j];
                if (!a.active && !b.active) continue;

                Vec3 diff = b.position - a.position;
                float dist = diff.length();
                float min_dist = a.radius + b.radius;

                if (dist < min_dist && dist > 1e-6f) {
                    Vec3 normal = diff / dist;
                    float overlap = min_dist - dist;

                    // separate
                    float a_mass = a.is_static ? 1e9f : std::fmax(a.mass, 1e-6f);
                    float b_mass = b.is_static ? 1e9f : std::fmax(b.mass, 1e-6f);
                    float total_mass = a_mass + b_mass;

                    if (!a.is_static) {
                        a.position -= normal * (overlap * b_mass / total_mass);
                    }
                    if (!b.is_static) {
                        b.position += normal * (overlap * a_mass / total_mass);
                    }

                    // relative velocity
                    Vec3 rel_vel = b.velocity - a.velocity;
                    float vel_along_normal = rel_vel.dot(normal);

                    if (vel_along_normal > 0) continue; // moving apart

                    // impulse
                    float restitution = (a.bounce + b.bounce) * 0.5f;
                    float impulse = -(1.0f + restitution) * vel_along_normal;
                    impulse /= (1.0f / a_mass + 1.0f / b_mass);

                    Vec3 impulse_vec = normal * impulse;
                    if (!a.is_static) {
                        a.velocity -= impulse_vec / a_mass;
                        // torque from off-center hit
                        Vec3 contact_a = normal * std::fmax(a.radius, 1e-6f);
                        Vec3 torque = contact_a.cross(impulse_vec);
                        a.angular_velocity += torque / (a_mass * std::fmax(a.radius, 1e-6f) * std::fmax(a.radius, 1e-6f) * 0.4f);
                    }
                    if (!b.is_static) {
                        b.velocity += impulse_vec / b_mass;
                        Vec3 contact_b = -normal * std::fmax(b.radius, 1e-6f);
                        Vec3 torque = contact_b.cross(-impulse_vec);
                        b.angular_velocity += torque / (b_mass * std::fmax(b.radius, 1e-6f) * std::fmax(b.radius, 1e-6f) * 0.4f);
                    }

                    // friction (tangential)
                    Vec3 tangent = rel_vel - normal * vel_along_normal;
                    float tangent_len = tangent.length();
                    if (tangent_len > 1e-6f) {
                        tangent = tangent / tangent_len;
                        float friction_impulse = tangent_len * 0.3f;
                        Vec3 friction_vec = tangent * friction_impulse;
                        if (!a.is_static) a.velocity -= friction_vec / a_mass;
                        if (!b.is_static) b.velocity += friction_vec / b_mass;
                    }
                }
            }
        }
    }
};

} // namespace pencil
