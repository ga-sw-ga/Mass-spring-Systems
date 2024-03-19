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
			: mass_geometry(givr::geometry::Radius(0.2f))
			, mass_style(givr::style::Colour(1.f, 0.f, 1.f), givr::style::LightPosition(100.f, 100.f, 100.f))
			, spring_geometry()
			, spring_style(givr::style::Colour(1.f, 0.f, 1.f))
		{
			// Link up (Static elements)
			mass_a.fixed = true;
			mass_b.fixed = false;
			spring.mass_a = &mass_a;
			spring.mass_b = &mass_b;
            spring.rest_l = 5.f;
            spring.k_s = 1.f;
            spring.k_d = 0.1f;
			// Reset Dynamic elements
			reset();

			// Render
			mass_render = givr::createInstancedRenderable(mass_geometry, mass_style);
			spring_render = givr::createRenderable(spring_geometry, spring_style);
		}

		void MassOnSpringModel::reset() {
			//As you add quantities to the primatives, they should be set here.
			mass_a.p = { 0.f,0.f,0.f };
			mass_a.v = { 0.f,0.f,0.f }; // Fixed anyway so doesnt matter if implemented correctly
			mass_b.p = { 0.f,-5.f,0.f };
			mass_b.v = { 0.f,-3.f,0.f };
			//This model can start vertical and be just a spring in the y direction only (like currently set up)
		}


        void MassOnSpringModel::step(float dt) {
            g = glm::vec3(0.f, -1.f * imgui_panel::gravity, 0.f);

            mass_b.f = spring.force_b() + mass_b.m * g;
            mass_b.integrate(dt);
        }


        void MassOnSpringModel::render(const ModelViewContext& view) {

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
			: mass_geometry(givr::geometry::Radius(0.2f))
			, mass_style(givr::style::Colour(1.f, 0.f, 1.f), givr::style::LightPosition(100.f, 100.f, 100.f))
			, spring_geometry()
			, spring_style(givr::style::Colour(1.f, 0.f, 1.f))
		{
			//TODO: This chain should be at least ***10*** links long!

            int number_of_masses = 10, number_of_springs = number_of_masses - 1;
			//Link up (Static elements)
			masses.resize(number_of_masses);
			masses[0].fixed = true;
            for (int i = 1; i < number_of_masses; ++i) {
                masses[i].fixed = false;
            }

			springs.resize(number_of_springs);
            for (int i = 0; i < number_of_springs; ++i) {
                springs[i].mass_a = &masses[i];
                springs[i].mass_b = &masses[i + 1];
                springs[i].k_s = 25.f;
                springs[i].k_d = 1.f;
                springs[i].rest_l = 5.f;
            }
			//Reset Dynamic elements
			reset();

			// Render
			mass_render = givr::createInstancedRenderable(mass_geometry, mass_style);
			spring_render = givr::createRenderable(spring_geometry, spring_style);
		}

		void ChainPendulumModel::reset() {
			//As you add quantities to the primatives, they should be set here.
            for (int i = 0; i < masses.size(); ++i) {
                masses[i].p = {(float)i * 5.f,0.f,0.f };
                masses[i].v = {0.f, 0.f, 0.f};
            }
			//The model should start non-vertical so we can see swaying action
		}

		void ChainPendulumModel::step(float dt) {
			//TODO: Complete the Chain Pendulum step
            //Calculating the forces
            g = glm::vec3(0.f, -1.f * imgui_panel::gravity, 0.f);

            for (int i = 0; i < masses.size(); ++i) {
                masses[i].f = masses[i].m * g;
            }
            for (int i = 0; i < springs.size(); ++i) {
                springs[i].mass_a->f += springs[i].force_a();
                springs[i].mass_b->f += springs[i].force_b();
            }

            //Integration
            for (int i = 0; i < masses.size(); ++i) {
                masses[i].integrate(dt);
            }
		}

		void ChainPendulumModel::render(const ModelViewContext& view) {

			//Add Mass render
			for (const primatives::Mass& mass : masses) {
				givr::addInstance(mass_render, glm::translate(glm::mat4(1.f), mass.p));
			}

			//Clear and add springs
			spring_geometry.segments().clear();
			for (const primatives::Spring& spring : springs) {
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
                : mass_geometry(givr::geometry::Radius(0.2f))
                , mass_style(givr::style::Colour(1.f, 0.f, 1.f), givr::style::LightPosition(100.f, 100.f, 100.f))
                , spring_geometry()
                , spring_style(givr::style::Colour(1.f, 0.f, 1.f))
        {

            //Initializing masses and springs
            int number_of_springs = 0;
            float k_d = 0.05f, k_s = 250.f;
            glm::vec3 center_of_jelly = glm::vec3((cube_size - 1) / 2.f) + offset;
            bool processed_masses[cube_size][cube_size][cube_size];
            for (int x = 0; x < cube_size; ++x) {
                for (int y = 0; y < cube_size; ++y) {
                    for (int z = 0; z < cube_size; ++z) {
                        processed_masses[x][y][z] = false;
                    }
                }
            }

            masses.resize(cube_size);
            for (int x = 0; x < cube_size; ++x) {
                masses[x].resize(cube_size);
                for (int y = 0; y < cube_size; ++y) {
                    masses[x][y].resize(cube_size);
                    for (int z = 0; z < cube_size; ++z) {
                        // Initialize each mass in the cubeOfJelly
                        masses[x][y][z].p = glm::vec3(x, y, z) * min_mass_distance + offset;
                        // Adding torque to the jelly
                        glm::vec3 vector = masses[x][y][z].p - center_of_jelly;
                        masses[x][y][z].v = glm::cross(glm::normalize(vector), glm::normalize(glm::vec3(1.f, 0.f, 1.f))) * torque_intensity;
                        for (int nx = 0; nx < cube_size; ++nx) {
                            for (int ny = 0; ny < cube_size; ++ny) {
                                for (int nz = 0; nz < cube_size; ++nz) {
                                    if (!(nx == x && ny == y && nz == z) && !processed_masses[nx][ny][nz]) {
                                        float distance = sqrtf((float)((nx - x)*(nx - x) + (ny - y)*(ny - y) + (nz - z)*(nz - z)));
                                        if (distance < min_mass_distance * 2.f ||
                                            (distance >= min_mass_distance * (cube_size - 1) - 0.01f &&
                                            distance < min_mass_distance * (cube_size - 1) + 0.01f))
                                        {
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
            for (int x = 0; x < cube_size; ++x) {
                for (int y = 0; y < cube_size; ++y) {
                    for (int z = 0; z < cube_size; ++z) {
                        processed_masses[x][y][z] = false;
                    }
                }
            }

            for (int x = 0; x < cube_size; ++x) {
                for (int y = 0; y < cube_size; ++y) {
                    for (int z = 0; z < cube_size; ++z) {
                        for (int nx = 0; nx < cube_size; ++nx) {
                            for (int ny = 0; ny < cube_size; ++ny) {
                                for (int nz = 0; nz < cube_size; ++nz) {
                                    if (!(nx == x && ny == y && nz == z) && !processed_masses[nx][ny][nz]) {
                                        float distance = sqrtf((float)((nx - x)*(nx - x) + (ny - y)*(ny - y) + (nz - z)*(nz - z)));
                                        if (distance < min_mass_distance * 2.f ||
                                            (distance >= min_mass_distance * (cube_size - 1) - 0.01f &&
                                             distance < min_mass_distance * (cube_size - 1) + 0.01f))
                                        {
                                            springs[spring_index].mass_a = &masses[x][y][z];
                                            springs[spring_index].mass_b = &masses[nx][ny][nz];
                                            // Set spring constants, rest length, etc. as needed
                                            springs[spring_index].rest_l = distance;
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


            //Reset Dynamic elements
            reset();

            // Render
            mass_render = givr::createInstancedRenderable(mass_geometry, mass_style);
            spring_render = givr::createRenderable(spring_geometry, spring_style);
        }

        void CubeOfJellyModel::reset() {
            glm::vec3 center_of_jelly = glm::vec3((cube_size - 1) / 2.f) + offset;
            for (int x = 0; x < masses.size(); ++x) {
                for (int y = 0; y < masses[x].size(); ++y) {
                    for (int z = 0; z < masses[x][y].size(); ++z) {
                        masses[x][y][z].p = glm::vec3(x, y, z) * min_mass_distance + offset;
                        glm::vec3 vector = masses[x][y][z].p - center_of_jelly;
                        masses[x][y][z].v = glm::cross(glm::normalize(vector), glm::normalize(glm::vec3(1.f, 0.f, 1.f))) * torque_intensity;
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
            float ground_height = -1.5f, ground_k_s = 100000.f, ground_k_d = 0.4f;
            for (int x = 0; x < masses.size(); ++x) {
                for (int y = 0; y < masses[x].size(); ++y) {
                    for (int z = 0; z < masses[x][y].size(); ++z) {
                        if (masses[x][y][z].p[1] < ground_height) {
                            float s_f = (ground_height - masses[x][y][z].p[1]) * ground_k_s;
                            float d_f = -1.f * masses[x][y][z].v[1] * ground_k_d;
                            masses[x][y][z].f += glm::vec3(0.f, s_f + d_f, 0.f);
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

        void CubeOfJellyModel::render(const ModelViewContext& view) {

            //Add Mass render
            for (const auto& layer : masses) {
                for (const auto& row : layer) {
                    for (const primatives::Mass& mass : row) {
                        givr::addInstance(mass_render, glm::translate(glm::mat4(1.f), mass.p));
                    }
                }
            }


            //Clear and add springs
            spring_geometry.segments().clear();
            for (const primatives::Spring& spring : springs) {
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
	} // namespace models
} // namespace simulation