#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D starToEndWithDiv = (end - start) / ((float)num_nodes - 1.f);
        Mass* StarMass = new Mass(start, node_mass, false);
        this->masses.emplace_back(StarMass);
        for (int i = 1; i < num_nodes; i++)
        {
            Vector2D point = start + starToEndWithDiv * i;
            Mass* pointMass = new Mass(point, node_mass, false);
            pointMass->velocity = Vector2D(0.f, 0.f);
            this->masses.emplace_back(pointMass);
            // std::vector<Mass* >::iterator it = this->masses.end();
            // Spring* spring = new Spring(*(it - 2), *(it - 1), k);
            // this->springs.emplace_back(spring);
            Spring* spring = new Spring(masses[i - 1], masses[i], k);
            this->springs.emplace_back(spring);
        }
        pinned_nodes.resize(num_nodes);

//        Comment-in this part when you implement the constructor
       for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;

       }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D dir = (s->m2->position - s->m1->position).unit();
            float f = s->k * ((s->m2->position - s->m1->position).norm() - s->rest_length);
            s->m1->forces += f * dir;
            s->m2->forces += -f * dir;
        }

        float kd = 0.00005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position


                // Vector2D a = m->forces / m->mass + gravity;
                // m->velocity += a * delta_t;
                // m->position += m->velocity * delta_t;


                // TODO (Part 2): Add global damping
                Vector2D a = m->forces / m->mass + gravity;
                m->velocity += a * delta_t - kd * m->velocity;
                m->position += m->velocity * delta_t;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D dir = (s->m2->position - s->m1->position).unit();
            float f = s->k * ((s->m2->position - s->m1->position).norm() - s->rest_length);
            s->m1->forces += f * dir;
            s->m2->forces += -f * dir;
        }
        float damping_factor = 0.00005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;

                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D a =  m->forces / m->mass + gravity;


                // m->position = temp_position + m->velocity * delta_t + a * delta_t * delta_t;
                // m->velocity += a * delta_t;


                // m->position = temp_position + (temp_position - m->last_position) + a * delta_t * delta_t;
                // m->last_position = temp_position;

                // TODO (Part 4): Add global Verlet damping

                m->position = temp_position + (1 - damping_factor) * (temp_position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
                
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
