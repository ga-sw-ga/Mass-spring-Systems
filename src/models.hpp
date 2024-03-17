#pragma once

#include <vector>
#include <givr.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

namespace simulation {
	namespace primatives {
		//Mass points used in all simulations
		struct Mass {
			bool fixed = false;
			glm::vec3 p = glm::vec3(0.f);
			glm::vec3 v = glm::vec3(0.f);
			//TO-DO: Modify this class to include certain desired quantities (mass, force, ...)
			//May even add functions! Such as integration ...
		};

		//Spring connections used in all simulations
		struct Spring {
			Mass* mass_a = nullptr;
			Mass* mass_b = nullptr;
			//TO-DO: Modify this class to include certain desired quantities (spring constants, rest length, ...)
			//May even add functions! Such as length, force, ...
		};

		//Face connections used (can just be a render primative or a simulation primatives for the bonus)
		struct Face {
			Mass* mass_a = nullptr;
			Mass* mass_b = nullptr;
			Mass* mass_c = nullptr;
		};
	} // namespace primatives

	namespace models {
		//If you want to use a different view, change this and the one in main
		using ModelViewContext = givr::camera::ViewContext<givr::camera::TurnTableCamera, givr::camera::PerspectiveProjection>;
		// Abstract class used by all models
		class GenericModel {
		public:
			virtual void reset() = 0;
			virtual void step(float dt) = 0;
			virtual void render(const ModelViewContext& view) = 0;
		};

		//Model constructing a single spring
		class MassOnSpringModel : public GenericModel {
		public:
			MassOnSpringModel();
			void reset();
			void step(float dt);
			void render(const ModelViewContext& view);

			//Simulation Constants (you can re-assign values here from imgui)
			glm::vec3 g = { 0.f, -9.81f, 0.f };

		private:
			//Simulation Parts
			primatives::Mass mass_a;
			primatives::Mass mass_b;
			primatives::Spring spring;

			//Render
			givr::geometry::Sphere mass_geometry; 
			givr::style::Phong mass_style;
			givr::InstancedRenderContext<givr::geometry::Sphere, givr::style::Phong> mass_render;

			givr::geometry::MultiLine spring_geometry;
			givr::style::LineStyle spring_style;
			givr::RenderContext<givr::geometry::MultiLine, givr::style::LineStyle> spring_render;
		};

		//Model constructing a chain of springs
		class ChainPendulumModel : public GenericModel {
		public:
			ChainPendulumModel();
			void reset();
			void step(float dt);
			void render(const ModelViewContext& view);

			//Simulation Constants (you can re-assign values here from imgui)
			glm::vec3 g = { 0.f, -9.81f, 0.f };

		private:

			//Simulation Parts
			std::vector<primatives::Mass> masses;
			std::vector<primatives::Spring> springs;

			//Render
			givr::geometry::Sphere mass_geometry;
			givr::style::Phong mass_style;
			givr::InstancedRenderContext<givr::geometry::Sphere, givr::style::Phong> mass_render;

			givr::geometry::MultiLine spring_geometry;
			givr::style::LineStyle spring_style;
			givr::RenderContext<givr::geometry::MultiLine, givr::style::LineStyle> spring_render;
		};

		// TO-DO: Fully implements the last two using the scheme provided above
		//class CubeOfJellyModel : public GenericModel {...}; //should be at least 4 in each direction
		//class HangingClothModel : public GenericModel {...}; //should be at least 8 in each direction

	} // namespace models
} // namespace simulation