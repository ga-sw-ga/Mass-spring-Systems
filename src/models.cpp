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
	} // namespace models
} // namespace simulation