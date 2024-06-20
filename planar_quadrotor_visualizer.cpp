#include "planar_quadrotor_visualizer.h"
#include <cmath>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr), blade_angle(0) {}

/**
 * TODO: Improve visualization
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate propellers (extra points)
 */
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();

    float q_x, q_y, q_theta;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    
    int body_center_x = static_cast<int>(q_x);
    int body_center_y = static_cast<int>(q_y);

    
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);

    //  rotation to the body
    float cos_theta = std::cos(-q_theta);
    float sin_theta = std::sin(-q_theta);

    //  body of the drone
    SDL_Point body[4];
    body[0] = { static_cast<int>(-15 * cos_theta - 10 * sin_theta + body_center_x), static_cast<int>(-15 * sin_theta + 10 * cos_theta + body_center_y) };
    body[1] = { static_cast<int>(15 * cos_theta - 10 * sin_theta + body_center_x), static_cast<int>(15 * sin_theta + 10 * cos_theta + body_center_y) };
    body[2] = { static_cast<int>(15 * cos_theta + 10 * sin_theta + body_center_x), static_cast<int>(15 * sin_theta - 10 * cos_theta + body_center_y) };
    body[3] = { static_cast<int>(-15 * cos_theta + 10 * sin_theta + body_center_x), static_cast<int>(-15 * sin_theta - 10 * cos_theta + body_center_y) };

    SDL_RenderDrawLines(gRenderer.get(), body, 4);
    SDL_RenderDrawLine(gRenderer.get(), body[3].x, body[3].y, body[0].x, body[0].y);

    //  arms of the drone
    SDL_RenderDrawLine(gRenderer.get(), body_center_x, body_center_y,
        body_center_x + static_cast<int>(50 * cos_theta),
        body_center_y + static_cast<int>(50 * sin_theta));
    SDL_RenderDrawLine(gRenderer.get(), body_center_x, body_center_y,
        body_center_x - static_cast<int>(50 * cos_theta),
        body_center_y - static_cast<int>(50 * sin_theta));

    //  propellers as circles
    filledCircleColor(gRenderer.get(),
        body_center_x + static_cast<int>(50 * cos_theta),
        body_center_y + static_cast<int>(50 * sin_theta),
        5, 0xFF0000FF);
    filledCircleColor(gRenderer.get(),
        body_center_x - static_cast<int>(50 * cos_theta),
        body_center_y - static_cast<int>(50 * sin_theta),
        5, 0xFF0000FF);

    // Windmill of the right circle
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
    int windmill_right_x = body_center_x + static_cast<int>(50 * cos_theta);
    int windmill_right_y = body_center_y + static_cast<int>(50 * sin_theta);
    int blade_length = 20;

    //  end of propellers on the right circle
    for (int i = 0; i < 4; ++i) {
        if (std::abs(std::cos(blade_angle)) > 0.8) {
            
            blade_length = 19; 
        }
        else {
            
            blade_length = 10; 
        }
        int blade_end_x = windmill_right_x + static_cast<int>(blade_length * std::cos(blade_angle));
        int blade_end_y = windmill_right_y + static_cast<int>(blade_length * std::sin(blade_angle));
        SDL_RenderDrawLine(gRenderer.get(), windmill_right_x, windmill_right_y, blade_end_x, blade_end_y);
        blade_angle += M_PI / 6; 
    }

    // Windmill  of the left circle
    int windmill_left_x = body_center_x - static_cast<int>(50 * cos_theta);
    int windmill_left_y = body_center_y - static_cast<int>(50 * sin_theta);

    //  end of propellers on the left circle
    for (int i = 0; i < 4; ++i) {
        if (std::abs(std::cos(blade_angle)) > 0.8) {
            
            blade_length = 19; 
        }
        else {
            
            blade_length = 10; 
        }
        int blade_end_x = windmill_left_x + static_cast<int>(blade_length * std::cos(blade_angle));
        int blade_end_y = windmill_left_y + static_cast<int>(blade_length * std::sin(blade_angle));
        SDL_RenderDrawLine(gRenderer.get(), windmill_left_x, windmill_left_y, blade_end_x, blade_end_y);
        blade_angle += M_PI / 6; 
    }
}
