#pragma once

#include <vector>
#include <givr.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "imgui_panel.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

namespace simulation {
	namespace primatives {
		//Mass points used in all simulations
		struct Mass {
			bool fixed = false;
            bool air_resistance = false;
			glm::vec3 p = glm::vec3(0.f);
			glm::vec3 v = glm::vec3(0.f);
            glm::vec3 f = glm::vec3(0.f);
            float m = 1.f;

            // Integration function
            void integrate(float dt) {
                if (!fixed) {
                    glm::vec3 a = f / m;
                    v += a * dt;
                    p += v * dt;
                }
            }

            // Verlet Integration (For using when necessary)
//            void integrate(float dt) {
//                if (!fixed) {
//                    glm::vec3 a = f / m;
//                    glm::vec3 prevP = p;
//                    p += (v + a * dt * 0.5f) * dt;
//                    v = (p - prevP) / dt;
//                }
//            }

        };

		//Spring connections used in all simulations
		struct Spring {
			Mass* mass_a = nullptr;
			Mass* mass_b = nullptr;
            float rest_l = 0.f;
            float k_s = 0.f;
            float k_d = 0.f;

            // Function to calculate spring length
            float length() const {
                return glm::length(mass_b->p - mass_a->p);
            }

            // Function to calculate spring force (applied on mass a)
            glm::vec3 force_a() const {
                glm::vec3 delta_p = mass_b->p - mass_a->p;
                float current_l = glm::length(delta_p);
                float displacement = current_l - rest_l;
                glm::vec3 force = k_s * displacement * glm::normalize(delta_p);
                glm::vec3 damping_force = k_d * (mass_a->v - mass_b->v);
                return force - damping_force;
            }

            // Function to calculate spring force (applied on mass b)
            glm::vec3 force_b() const {
                return force_a() * -1.f;
            }
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
            float c_d = 0.05f;

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
            float c_d = 0.005f;

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

        class CubeOfJellyModel : public GenericModel {
        public:
            CubeOfJellyModel();
            void reset();
            void step(float dt);
            void render(const ModelViewContext& view);

            //Simulation Constants (you can re-assign values here from imgui)
            glm::vec3 g = { 0.f, -9.81f, 0.f };
            float c_d = 0.005f;
            glm::vec3 offset = { 0.f,  10.f, 0.f };
            int cube_width = 7, cube_height = 6, cube_depth = 5;
            float min_mass_distance = 1.f, torque_intensity = 15.f, ground_height = -1.5f;
            std::vector<primatives::Face> faces;

        private:

            //Simulation Parts
            std::vector<std::vector<std::vector<primatives::Mass>>> masses;
            std::vector<primatives::Spring> springs;

            //Render
            givr::geometry::TriangleSoup triangle_geometry;
            givr::style::Phong triangle_style;
            givr::RenderContext<givr::geometry::TriangleSoup, givr::style::Phong> triangle_render;

            givr::geometry::Quad ground_geometry;
            givr::style::Phong ground_style;
            givr::RenderContext<givr::geometry::Quad, givr::style::Phong> ground_render;
        };

        class HangingClothModel : public GenericModel {
        public:
            HangingClothModel();
            void reset();
            void step(float dt);
            void render(const ModelViewContext& view);

            //Simulation Constants (you can re-assign values here from imgui)
            glm::vec3 g = { 0.f, -9.81f, 0.f };
            float c_d = 0.0025f;
            int width = 30, height = 40;
            float min_mass_distance = 1.f;
            std::vector<primatives::Face> faces;

        private:

            //Simulation Parts
            std::vector<std::vector<primatives::Mass>> masses;
            std::vector<primatives::Spring> springs;

            //Render
            givr::geometry::TriangleSoup triangle_geometry;
            givr::style::Phong triangle_style;
            givr::RenderContext<givr::geometry::TriangleSoup, givr::style::Phong> triangle_render;
        };
    } // namespace models
} // namespace simulation