#include <cstdlib>
#include <cmath>

#include "models.hpp"
#include "imgui_panel.hpp"

namespace simulation {
    namespace primatives {
        //No functions for structs, yet...
    }// namespace primatives

    namespace models {
        //////////////////////////////////////////////////
        ////            MassOnSpringModel             ////----------------------------------------------------------
        //////////////////////////////////////////////////

        MassOnSpringModel::MassOnSpringModel()
                : mass_geometry(givr::geometry::Radius(0.2f)),
                  mass_style(givr::style::Colour(1.f, 0.f, 1.f), givr::style::LightPosition(100.f, 100.f, 100.f)),
                  spring_geometry(), spring_style(givr::style::Colour(1.f, 0.f, 1.f)) {
            // Link up (Static elements)
            mass_a.fixed = true;
            mass_b.fixed = false;
            spring.mass_a = &mass_a;
            spring.mass_b = &mass_b;
            spring.rest_l = 5.f;
            spring.k_s = 2.f;
            spring.k_d = 0.1f;
            // Reset Dynamic elements
            reset();

            // Render
            mass_render = givr::createInstancedRenderable(mass_geometry, mass_style);
            spring_render = givr::createRenderable(spring_geometry, spring_style);
        }

        void MassOnSpringModel::reset() {
            mass_a.p = {0.f, 0.f, 0.f};
            mass_a.v = {0.f, 0.f, 0.f};
            mass_b.p = {0.f, -5.f, 0.f};
            mass_b.v = {0.f, -3.f, 0.f};
        }


        void MassOnSpringModel::step(float dt) {
            g = glm::vec3(0.f, -1.f * imgui_panel::gravity, 0.f);

            mass_b.f = spring.force_b() + mass_b.m * g;
            mass_b.integrate(dt);
        }


        void MassOnSpringModel::render(const ModelViewContext &view) {

            //Add Mass render
            givr::addInstance(mass_render, glm::translate(glm::mat4(1.f), mass_a.p));
            givr::addInstance(mass_render, glm::translate(glm::mat4(1.f), mass_b.p));

            //Clear and add springs
            spring_geometry.segments().clear();
            spring_geometry.push_back(
                    givr::geometry::Line(
                            givr::geometry::Point1(spring.mass_a->p),
                            givr::geometry::Point2(spring.mass_b->p)
                    )
            );
            givr::updateRenderable(spring_geometry, spring_style, spring_render);

            //Render
            givr::style::draw(mass_render, view);
            givr::style::draw(spring_render, view);
        }

        //////////////////////////////////////////////////
        ////           ChainPendulumModel             ////----------------------------------------------------------
        //////////////////////////////////////////////////

        ChainPendulumModel::ChainPendulumModel()
                : mass_geometry(givr::geometry::Radius(0.2f)),
                  mass_style(givr::style::Colour(1.f, 0.f, 1.f), givr::style::LightPosition(100.f, 100.f, 100.f)),
                  spring_geometry(), spring_style(givr::style::Colour(1.f, 0.f, 1.f)) {

            int number_of_masses = 20, number_of_springs = number_of_masses - 1;
            //Link up (Static elements)
            masses.resize(number_of_masses);
            masses[0].fixed = true;
            for (int i = 0; i < number_of_masses; ++i) {
                masses[i].air_resistance = true;
            }

            springs.resize(number_of_springs);
            for (int i = 0; i < number_of_springs; ++i) {
                springs[i].mass_a = &masses[i];
                springs[i].mass_b = &masses[i + 1];
                springs[i].k_s = 1000.f;
                springs[i].k_d = 5.f;
                springs[i].rest_l = 1.f;
            }
            //Reset Dynamic elements
            reset();

            // Render
            mass_render = givr::createInstancedRenderable(mass_geometry, mass_style);
            spring_render = givr::createRenderable(spring_geometry, spring_style);
        }

        void ChainPendulumModel::reset() {
            for (int i = 0; i < masses.size(); ++i) {
                masses[i].p = {(float) i * 1.f, 0.f, 0.f};
                masses[i].v = {0.f, 0.f, 0.f};
            }
        }

        void ChainPendulumModel::step(float dt) {
            //Calculating the forces
            g = glm::vec3(0.f, -1.f * imgui_panel::gravity, 0.f);

            for (int i = 0; i < masses.size(); ++i) {
                masses[i].f = masses[i].m * g;
            }
            for (int i = 0; i < springs.size(); ++i) {
                springs[i].mass_a->f += springs[i].force_a();
                springs[i].mass_b->f += springs[i].force_b();
            }
            for (int i = 0; i < masses.size(); ++i) {
                if (masses[i].air_resistance) {
                    if (glm::length(masses[i].v) > 0.f) {
                        masses[i].f += -1.f * glm::dot(masses[i].v, masses[i].v) * c_d * glm::normalize(masses[i].v);
                    }
                }
            }

            //Integration
            for (int i = 0; i < masses.size(); ++i) {
                masses[i].integrate(dt);
            }
        }

        void ChainPendulumModel::render(const ModelViewContext &view) {

            //Add Mass render
            for (const primatives::Mass &mass: masses) {
                givr::addInstance(mass_render, glm::translate(glm::mat4(1.f), mass.p));
            }

            //Clear and add springs
            spring_geometry.segments().clear();
            for (const primatives::Spring &spring: springs) {
                spring_geometry.push_back(
                        givr::geometry::Line(
                                givr::geometry::Point1(spring.mass_a->p),
                                givr::geometry::Point2(spring.mass_b->p)
                        )
                );
            }
            givr::updateRenderable(spring_geometry, spring_style, spring_render);

            //Render
            givr::style::draw(mass_render, view);
            givr::style::draw(spring_render, view);
        }

        //////////////////////////////////////////////////
        ////           CubeOfJellyModel             ////----------------------------------------------------------
        //////////////////////////////////////////////////

        CubeOfJellyModel::CubeOfJellyModel()
                : triangle_geometry(),
                  triangle_style(givr::style::Colour(1.f, 0.f, 1.f), givr::style::LightPosition(100.f, 100.f, 100.f),
                                 givr::style::AmbientFactor(0.3f)),
                  ground_geometry(givr::geometry::Point1(glm::vec3(-100.f, ground_height, -100.f)),
                                  givr::geometry::Point2(glm::vec3(-100.f, ground_height, 100.f)),
                                  givr::geometry::Point3(glm::vec3(100.f, ground_height, 100.f)),
                                  givr::geometry::Point4(glm::vec3(100.f, ground_height, -100.f))),
                  ground_style(givr::style::Colour(0.25f, 0.25f, 1.f), givr::style::LightPosition(0.f, 100.f, 0.f)) {

            //Initializing masses and springs
            int number_of_springs = 0;
            float k_d = 0.05f, k_s = 250.f;
            bool processed_masses[cube_width][cube_height][cube_depth];
            for (int x = 0; x < cube_width; ++x) {
                for (int y = 0; y < cube_height; ++y) {
                    for (int z = 0; z < cube_depth; ++z) {
                        processed_masses[x][y][z] = false;
                    }
                }
            }

            masses.resize(cube_width);
            for (int x = 0; x < cube_width; ++x) {
                masses[x].resize(cube_height);
                for (int y = 0; y < cube_height; ++y) {
                    masses[x][y].resize(cube_depth);
                    for (int z = 0; z < cube_depth; ++z) {
                        masses[x][y][z].air_resistance = true;
                        for (int nx = 0; nx < cube_width; ++nx) {
                            for (int ny = 0; ny < cube_height; ++ny) {
                                for (int nz = 0; nz < cube_depth; ++nz) {
                                    if (!(nx == x && ny == y && nz == z) && !processed_masses[nx][ny][nz]) {
                                        float distance = sqrtf((float) ((nx - x) * (nx - x) + (ny - y) * (ny - y) +
                                                                        (nz - z) * (nz - z)) * min_mass_distance);
                                        if (distance < min_mass_distance * 2.f ||
                                            (distance >= min_mass_distance * (cube_width - 1) - 0.01f &&
                                             distance < min_mass_distance * (cube_width - 1) + 0.01f) ||
                                            (distance >= min_mass_distance * (cube_height - 1) - 0.01f &&
                                             distance < min_mass_distance * (cube_height - 1) + 0.01f) ||
                                            (distance >= min_mass_distance * (cube_depth - 1) - 0.01f &&
                                             distance < min_mass_distance * (cube_depth - 1) + 0.01f)) {
                                            number_of_springs++;
                                        }
                                    }
                                }
                            }
                        }

                        processed_masses[x][y][z] = true;
                    }
                }
            }

            springs.resize(number_of_springs);
            int spring_index = 0;
            for (int x = 0; x < cube_width; ++x) {
                for (int y = 0; y < cube_height; ++y) {
                    for (int z = 0; z < cube_depth; ++z) {
                        processed_masses[x][y][z] = false;
                    }
                }
            }

            for (int x = 0; x < cube_width; ++x) {
                for (int y = 0; y < cube_height; ++y) {
                    for (int z = 0; z < cube_depth; ++z) {
                        for (int nx = 0; nx < cube_width; ++nx) {
                            for (int ny = 0; ny < cube_height; ++ny) {
                                for (int nz = 0; nz < cube_depth; ++nz) {
                                    if (!(nx == x && ny == y && nz == z) && !processed_masses[nx][ny][nz]) {
                                        float distance = sqrtf((float) ((nx - x) * (nx - x) + (ny - y) * (ny - y) +
                                                                        (nz - z) * (nz - z)) * min_mass_distance);
                                        if (distance < min_mass_distance * 2.f ||
                                            (distance >= min_mass_distance * (cube_width - 1) - 0.01f &&
                                             distance < min_mass_distance * (cube_width - 1) + 0.01f) ||
                                            (distance >= min_mass_distance * (cube_height - 1) - 0.01f &&
                                             distance < min_mass_distance * (cube_height - 1) + 0.01f) ||
                                            (distance >= min_mass_distance * (cube_depth - 1) - 0.01f &&
                                             distance < min_mass_distance * (cube_depth - 1) + 0.01f)) {
                                            springs[spring_index].mass_a = &masses[x][y][z];
                                            springs[spring_index].mass_b = &masses[nx][ny][nz];
                                            // Set spring constants, rest length, etc. as needed
                                            springs[spring_index].rest_l = min_mass_distance * distance;
                                            springs[spring_index].k_s = k_s; // Example spring constant
                                            springs[spring_index].k_d = k_d; // Example damping constant
                                            spring_index++;
                                        }
                                    }
                                }
                            }
                        }
                        processed_masses[x][y][z] = true;
                    }
                }
            }

            triangle_render = givr::createRenderable(triangle_geometry, triangle_style);

            faces.resize((cube_width - 1) * (cube_height - 1) * 4 + (cube_height - 1) * (cube_depth - 1) * 4 +
                         (cube_width - 1) * (cube_depth - 1) * 4);
            int face_index = 0;
            for (int i = 0; i < 6; ++i) {
                int x_bound = (i < 2 ? cube_height : cube_width) - 1;
                int y_bound = (i > 3 ? cube_height : cube_depth) - 1;
                for (int j = 0; j < x_bound; ++j) {
                    for (int k = 0; k < y_bound; ++k) {
                        if (i == 0) {
                            faces[face_index].mass_a = &masses[0][j][k];
                            faces[face_index].mass_b = &masses[0][j + 1][k];
                            faces[face_index].mass_c = &masses[0][j][k + 1];
                            face_index++;
                            faces[face_index].mass_a = &masses[0][j + 1][k + 1];
                            faces[face_index].mass_b = &masses[0][j][k + 1];
                            faces[face_index].mass_c = &masses[0][j + 1][k];
                            face_index++;
                        } else if (i == 1) {
                            faces[face_index].mass_a = &masses[cube_width - 1][j][k];
                            faces[face_index].mass_b = &masses[cube_width - 1][j + 1][k];
                            faces[face_index].mass_c = &masses[cube_width - 1][j][k + 1];
                            face_index++;
                            faces[face_index].mass_a = &masses[cube_width - 1][j + 1][k + 1];
                            faces[face_index].mass_b = &masses[cube_width - 1][j][k + 1];
                            faces[face_index].mass_c = &masses[cube_width - 1][j + 1][k];
                            face_index++;
                        } else if (i == 2) {
                            faces[face_index].mass_a = &masses[j][0][k];
                            faces[face_index].mass_b = &masses[j + 1][0][k];
                            faces[face_index].mass_c = &masses[j][0][k + 1];
                            face_index++;
                            faces[face_index].mass_a = &masses[j + 1][0][k + 1];
                            faces[face_index].mass_b = &masses[j][0][k + 1];
                            faces[face_index].mass_c = &masses[j + 1][0][k];
                            face_index++;
                        } else if (i == 3) {
                            faces[face_index].mass_a = &masses[j][cube_height - 1][k];
                            faces[face_index].mass_b = &masses[j + 1][cube_height - 1][k];
                            faces[face_index].mass_c = &masses[j][cube_height - 1][k + 1];
                            face_index++;
                            faces[face_index].mass_a = &masses[j + 1][cube_height - 1][k + 1];
                            faces[face_index].mass_b = &masses[j][cube_height - 1][k + 1];
                            faces[face_index].mass_c = &masses[j + 1][cube_height - 1][k];
                            face_index++;
                        } else if (i == 4) {
                            faces[face_index].mass_a = &masses[j][k][0];
                            faces[face_index].mass_b = &masses[j + 1][k][0];
                            faces[face_index].mass_c = &masses[j][k + 1][0];
                            face_index++;
                            faces[face_index].mass_a = &masses[j + 1][k + 1][0];
                            faces[face_index].mass_b = &masses[j][k + 1][0];
                            faces[face_index].mass_c = &masses[j + 1][k][0];
                            face_index++;
                        } else if (i == 5) {
                            faces[face_index].mass_a = &masses[j][k][cube_depth - 1];
                            faces[face_index].mass_b = &masses[j + 1][k][cube_depth - 1];
                            faces[face_index].mass_c = &masses[j][k + 1][cube_depth - 1];
                            face_index++;
                            faces[face_index].mass_a = &masses[j + 1][k + 1][cube_depth - 1];
                            faces[face_index].mass_b = &masses[j][k + 1][cube_depth - 1];
                            faces[face_index].mass_c = &masses[j + 1][k][cube_depth - 1];
                            face_index++;
                        }
                    }
                }
            }

            //Reset Dynamic elements
            reset();

            // Render
            ground_render = givr::createRenderable(ground_geometry, ground_style);
        }

        void CubeOfJellyModel::reset() {
            glm::vec3 center_of_jelly =
                    glm::vec3((cube_width - 1) / 2.f, (cube_height - 1) / 2.f, (cube_depth - 1) / 2.f) + offset;
            for (int x = 0; x < masses.size(); ++x) {
                for (int y = 0; y < masses[x].size(); ++y) {
                    for (int z = 0; z < masses[x][y].size(); ++z) {
                        // Initialize each mass in the cubeOfJelly
                        masses[x][y][z].p = glm::vec3(x, y, z) * min_mass_distance + offset;
                        // Adding torque to the jelly
                        glm::vec3 vector = masses[x][y][z].p - center_of_jelly;
                        masses[x][y][z].v =
                                glm::cross(glm::normalize(vector), glm::normalize(glm::vec3(1.f, 0.7f, 0.5f))) *
                                torque_intensity;
                    }
                }
            }
        }

        void CubeOfJellyModel::step(float dt) {
            g = glm::vec3(0.f, -1.f * imgui_panel::gravity, 0.f);

            for (int x = 0; x < masses.size(); ++x) {
                for (int y = 0; y < masses[x].size(); ++y) {
                    for (int z = 0; z < masses[x][y].size(); ++z) {
                        masses[x][y][z].f = masses[x][y][z].m * g;
                    }
                }
            }

            for (int i = 0; i < springs.size(); ++i) {
                springs[i].mass_a->f += springs[i].force_a();
                springs[i].mass_b->f += springs[i].force_b();
            }

            // Handling collisions
            float ground_k_s = 100000.f, ground_k_d = 0.4f;
            for (int x = 0; x < masses.size(); ++x) {
                for (int y = 0; y < masses[x].size(); ++y) {
                    for (int z = 0; z < masses[x][y].size(); ++z) {
                        if (masses[x][y][z].p[1] < ground_height) {
                            float s_f = (ground_height - masses[x][y][z].p[1]) * ground_k_s;
                            float d_f = -1.f * masses[x][y][z].v[1] * ground_k_d;
                            masses[x][y][z].f += glm::vec3(0.f, s_f + d_f, 0.f);
                            if (masses[x][y][z].air_resistance) {
                                if (glm::length(masses[x][y][z].v) > 0.f) {
                                    masses[x][y][z].f += -1.f * glm::dot(masses[x][y][z].v, masses[x][y][z].v) * c_d *
                                                         glm::normalize(masses[x][y][z].v);
                                }
                            }
                        }
                    }
                }
            }

            //Integration
            for (int x = 0; x < masses.size(); ++x) {
                for (int y = 0; y < masses[x].size(); ++y) {
                    for (int z = 0; z < masses[x][y].size(); ++z) {
                        masses[x][y][z].integrate(dt);
                    }
                }
            }
        }

        void CubeOfJellyModel::render(const ModelViewContext &view) {
            triangle_geometry.triangles().clear();

            for (auto face: faces) {
                triangle_geometry.push_back(
                        givr::geometry::Triangle(givr::geometry::Point1(face.mass_a->p),
                                                 givr::geometry::Point2(face.mass_b->p),
                                                 givr::geometry::Point3(face.mass_c->p))
                );
            }

            givr::updateRenderable(triangle_geometry, triangle_style, triangle_render);

            givr::style::draw(triangle_render, view);
            givr::style::draw(ground_render, view);
        }

        //////////////////////////////////////////////////
        ////           HangingClothModel             ////----------------------------------------------------------
        //////////////////////////////////////////////////

        HangingClothModel::HangingClothModel()
                : triangle_geometry(),
                  triangle_style(givr::style::Colour(1.f, 0.f, 1.f), givr::style::LightPosition(100.f, 100.f, 100.f)) {

            //Initializing masses and springs
            int number_of_springs;
            float k_d = 0.5f, k_s = 5000.f;

            masses.resize(width + 1);
            for (int x = 0; x <= width; ++x) {
                masses[x].resize(height + 1);
                for (int y = 0; y <= height; ++y) {
                    masses[x][y].air_resistance = true;
                }
            }
            masses[0][0].fixed = true;
            masses[width][0].fixed = true;

            //Reset Dynamic elements
            reset();

            //Setting springs
            number_of_springs = width * height * 4 + width + height;
            springs.resize(number_of_springs);
            int spring_index = 0;
            for (int x = 0; x <= width; ++x) {
                for (int y = 0; y <= height; ++y) {
                    for (int dx = 0; dx <= 1; ++dx) {
                        for (int dy = 0; dy <= 1; ++dy) {
                            int xa, ya, xb, yb;
                            if (dx == 0 && dy == 0) {
                                xa = x + 1;
                                ya = y;
                                xb = x;
                                yb = y + 1;
                            } else {
                                xa = x;
                                ya = y;
                                xb = x + dx;
                                yb = y + dy;
                            }
                            if (xa <= width && xb <= width && ya <= height && yb <= height) {
                                springs[spring_index].mass_a = &masses[xa][ya];
                                springs[spring_index].mass_b = &masses[xb][yb];
                                springs[spring_index].rest_l = glm::distance(springs[spring_index].mass_a->p,
                                                                             springs[spring_index].mass_b->p);
                                springs[spring_index].k_s = k_s;
                                springs[spring_index].k_d = k_d;
                                spring_index++;
                            }
                        }
                    }
                }
            }

            // Render
            triangle_render = givr::createRenderable(triangle_geometry, triangle_style);

            faces.resize(width * height * 2);
            int face_index = 0;
            for (int x = 0; x < width; ++x) {
                for (int y = 0; y < height; ++y) {
                    for (int d = 0; d < 2; ++d) {
                        faces[face_index].mass_a = &masses[x + d][y + d];
                        faces[face_index].mass_b = &masses[x + 1 - d][y + d];
                        faces[face_index].mass_c = &masses[x + d][y + 1 - d];
                        face_index++;
                    }
                }
            }
        }

        void HangingClothModel::reset() {
            for (int x = 0; x <= width; ++x) {
                for (int y = 0; y <= height; ++y) {
                    masses[x][y].p = glm::vec3(((width * -0.5f) + x) * min_mass_distance, 0.f,
                                               ((height * -0.5f) + y) * min_mass_distance);
                    masses[x][y].v = glm::vec3(0.f);
                }
            }
        }

        void HangingClothModel::step(float dt) {
            g = glm::vec3(0.f, -1.f * imgui_panel::gravity, 0.f);

            for (int x = 0; x < masses.size(); ++x) {
                for (int y = 0; y < masses[x].size(); ++y) {
                    masses[x][y].f = masses[x][y].m * g;
                    if (masses[x][y].air_resistance) {
                        if (glm::length(masses[x][y].v) > 0.f) {
                            masses[x][y].f += -1.f * glm::dot(masses[x][y].v, masses[x][y].v) * c_d *
                                              glm::normalize(masses[x][y].v);
                        }
                    }
                }
            }

            for (int i = 0; i < springs.size(); ++i) {
                springs[i].mass_a->f += springs[i].force_a();
                springs[i].mass_b->f += springs[i].force_b();
            }

            //Integration
            for (int x = 0; x < masses.size(); ++x) {
                for (int y = 0; y < masses[x].size(); ++y) {
                    masses[x][y].integrate(dt);
                }
            }
        }

        // Calculate normals for each vertex of the cloth
        // Assuming you have a function to calculate normals, replace this with your actual normal calculation method
        glm::vec3 calculateNormal(primatives::Face face) {
            return glm::normalize(glm::cross(face.mass_b->p - face.mass_a->p, face.mass_c->p - face.mass_a->p));
        }

        void HangingClothModel::render(const ModelViewContext &view) {
            triangle_geometry.triangles().clear();

            for (auto face: faces) {
                triangle_geometry.push_back(
                        givr::geometry::Triangle(givr::geometry::Point1(face.mass_a->p),
                                                 givr::geometry::Point2(face.mass_b->p),
                                                 givr::geometry::Point3(face.mass_c->p))
                );
            }

            givr::updateRenderable(triangle_geometry, triangle_style, triangle_render);
            givr::style::draw(triangle_render, view);
        }
    } // namespace models
} // namespace simulation