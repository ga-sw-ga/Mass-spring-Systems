#include "models.hpp"

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
			mass_b.v = { 0.f,-1.f,0.f };
			//This model can start vertical and be just a spring in the y direction only (like currently set up)
		}

		void MassOnSpringModel::step(float dt) {
			//TODO: Just a moving mass, need to modify this to actually use spring
			mass_b.p += mass_b.v * dt;
			if (mass_b.p.y < -7.f) {
				mass_b.p.y = -7.f;
				mass_b.v = { 0.f, 0.2f,0.f };
			}
			else if (mass_b.p.y > -3.f) {
				mass_b.p.y = -3.f;
				mass_b.v = { 0.f, -0.2f ,0.f };
			}
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
			//Link up (Static elements)
			masses.resize(3);
			masses[0].fixed = true;
			masses[1].fixed = false;
			masses[2].fixed = false;

			springs.resize(2);
			springs[0].mass_a = &masses[0];
			springs[0].mass_b = &masses[1];
			springs[1].mass_a = &masses[1];
			springs[1].mass_b = &masses[2];
			//Reset Dynamic elements
			reset();

			// Render
			mass_render = givr::createInstancedRenderable(mass_geometry, mass_style);
			spring_render = givr::createRenderable(spring_geometry, spring_style);
		}

		void ChainPendulumModel::reset() {
			//As you add quantities to the primatives, they should be set here.
			masses[0].p = { 0.f,0.f,0.f };
			masses[0].v = { 0.f,0.f,0.f }; // Fixed anyway so doesnt matter if implemented correctly
			masses[1].p = { 5.f,0.f,0.f };
			masses[1].v = { 0.f,0.f,0.f };
			masses[2].p = { 10.f,0.f,0.f };
			masses[2].v = { 0.f,0.f,0.f };
			//The model should start non-vertical so we can see swaying action
		}

		void ChainPendulumModel::step(float dt) {
			//TODO: Complete the Chain Pendulum step
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